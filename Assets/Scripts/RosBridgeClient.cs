// RosBridgeClient.cs -- minimal rosbridge v2 client (JSON over WebSocket) for ROS 2's rosbridge_server.
//
// Same protocol as ROSBridgeLib (github.com/MathiasCiarlo/ROSBridgeLib), but with no DLLs (it uses .NET's
// ClientWebSocket) and ROS 2 message layouts (see RosMsgs.cs). ROS side:
//   ros2 launch rosbridge_server rosbridge_websocket_launch.xml      (port 9090)
//
// Network I/O runs on background tasks and reconnects automatically (advertisements and subscriptions are
// re-sent). Subscription callbacks run on the Unity main thread, from Poll().
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

public class RosBridgeClient : IDisposable
{
    public string Url { get; }
    public bool Connected { get; private set; }
    public string LastError { get; private set; }
    public int Pending => outbox.Count;   // messages waiting to be sent

    [Serializable] class Envelope { public string op; public string topic; }

    readonly Dictionary<string, string> advertised = new Dictionary<string, string>();   // topic -> type
    readonly Dictionary<string, string> subscribed = new Dictionary<string, string>();   // topic -> type
    readonly Dictionary<string, Action<string>> callbacks = new Dictionary<string, Action<string>>();
    readonly object topicsLock = new object();
    readonly ConcurrentQueue<string> outbox = new ConcurrentQueue<string>();
    readonly SemaphoreSlim outboxSignal = new SemaphoreSlim(0);
    readonly ConcurrentQueue<KeyValuePair<string, string>> inbox = new ConcurrentQueue<KeyValuePair<string, string>>();
    readonly CancellationTokenSource cts = new CancellationTokenSource();

    public RosBridgeClient(string url)
    {
        Url = url;
        Task.Run(() => RunAsync(cts.Token));
    }

    // type: ROS 2 name, e.g. "sensor_msgs/msg/Image"
    public void Advertise(string topic, string type)
    {
        lock (topicsLock) advertised[topic] = type;
        if (Connected) Enqueue(AdvertiseOp(topic, type));
    }

    // callback receives the full JSON of the incoming {"op":"publish",...} message, on the main thread
    public void Subscribe(string topic, string type, Action<string> callback)
    {
        lock (topicsLock) { subscribed[topic] = type; callbacks[topic] = callback; }
        if (Connected) Enqueue(SubscribeOp(topic, type));
    }

    // msg: a [Serializable] object from RosMsgs. Dropped (returns false) while disconnected.
    public bool Publish(string topic, object msg)
    {
        if (!Connected) return false;
        Enqueue("{\"op\":\"publish\",\"topic\":\"" + topic + "\",\"msg\":" + JsonUtility.ToJson(msg) + "}");
        return true;
    }

    // Call once per frame from the main thread: runs the subscription callbacks.
    public void Poll()
    {
        while (inbox.TryDequeue(out var m))
        {
            Action<string> cb;
            lock (topicsLock) callbacks.TryGetValue(m.Key, out cb);
            try { cb?.Invoke(m.Value); }
            catch (Exception e) { Debug.LogException(e); }
        }
    }

    public void Dispose()
    {
        cts.Cancel();
    }

    // ------------------------------------------------------------------ //
    static string AdvertiseOp(string topic, string type)
    {
        return "{\"op\":\"advertise\",\"topic\":\"" + topic + "\",\"type\":\"" + type + "\"}";
    }

    static string SubscribeOp(string topic, string type)
    {
        return "{\"op\":\"subscribe\",\"topic\":\"" + topic + "\",\"type\":\"" + type + "\",\"queue_length\":1}";
    }

    void Enqueue(string json)
    {
        outbox.Enqueue(json);
        outboxSignal.Release();
    }

    async Task RunAsync(CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            using (var ws = new ClientWebSocket())
            {
                try
                {
                    await ws.ConnectAsync(new Uri(Url), ct);
                    while (outbox.TryDequeue(out _)) { }   // drop anything stale from the last connection
                    List<string> hello = new List<string>();
                    lock (topicsLock)
                    {
                        foreach (var kv in advertised) hello.Add(AdvertiseOp(kv.Key, kv.Value));
                        foreach (var kv in subscribed) hello.Add(SubscribeOp(kv.Key, kv.Value));
                    }
                    foreach (var s in hello) await SendAsync(ws, s, ct);
                    Connected = true;
                    LastError = null;
                    Debug.Log($"[rosbridge] connected to {Url}");

                    using (var linked = CancellationTokenSource.CreateLinkedTokenSource(ct))
                    {
                        Task send = SendLoopAsync(ws, linked.Token), recv = ReceiveLoopAsync(ws, linked.Token);
                        Task done = await Task.WhenAny(send, recv);
                        linked.Cancel();
                        await done;   // rethrows the error that ended the connection
                    }
                }
                catch (OperationCanceledException) when (ct.IsCancellationRequested) { }
                catch (Exception e)
                {
                    if (Connected) Debug.LogWarning($"[rosbridge] connection lost: {e.Message}");
                    LastError = e.Message;
                }
                Connected = false;
                if (ws.State == WebSocketState.Open)
                {
                    try { await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "bye", CancellationToken.None); }
                    catch (Exception) { }
                }
            }
            try { await Task.Delay(1000, ct); } catch (OperationCanceledException) { }
        }
    }

    async Task SendLoopAsync(ClientWebSocket ws, CancellationToken ct)
    {
        while (true)
        {
            await outboxSignal.WaitAsync(ct);
            if (outbox.TryDequeue(out var s)) await SendAsync(ws, s, ct);
        }
    }

    static Task SendAsync(ClientWebSocket ws, string json, CancellationToken ct)
    {
        var bytes = Encoding.UTF8.GetBytes(json);
        return ws.SendAsync(new ArraySegment<byte>(bytes), WebSocketMessageType.Text, true, ct);
    }

    async Task ReceiveLoopAsync(ClientWebSocket ws, CancellationToken ct)
    {
        var buf = new byte[64 * 1024];
        var msg = new System.IO.MemoryStream();
        while (true)
        {
            var r = await ws.ReceiveAsync(new ArraySegment<byte>(buf), ct);
            if (r.MessageType == WebSocketMessageType.Close) throw new WebSocketException("closed by rosbridge");
            msg.Write(buf, 0, r.Count);
            if (!r.EndOfMessage) continue;
            string json = Encoding.UTF8.GetString(msg.GetBuffer(), 0, (int)msg.Length);
            msg.SetLength(0);
            var env = JsonUtility.FromJson<Envelope>(json);
            if (env != null && env.op == "publish" && env.topic != null)
                inbox.Enqueue(new KeyValuePair<string, string>(env.topic, json));
            else if (env != null && env.op == "status")
                Debug.LogWarning($"[rosbridge] {json}");
        }
    }
}
