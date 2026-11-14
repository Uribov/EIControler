// EIGloveDriver —— 把 EI Controller 手套的数据套到 VaM 的人体骨骼上
//
// 安装：放到 VaM 的 Custom/Scripts 目录，在场景里 Add Plugin 选它。
// 数据来源见 docs/vam-协议.md，上位机用 tools/eic_bridge.py。
//
// 协议解析逻辑在本文件底部（EIProtocol 类）。同样一份逻辑在
// EIGloveProtocol.cs 里有一个独立副本，用于 Linux 上跑单元测试。
// 改解析规则时两处要同步。

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

namespace EIGlove
{
    public class EIGloveDriver : MVRScript
    {
        const int UDP_PORT = 9001;

        const int JointCount = 16;

        static readonly string[] JointNames = {
            "hand",
            "thumb1", "thumb2", "thumb3",
            "index1", "index2", "index3",
            "middle1", "middle2", "middle3",
            "ring1", "ring2", "ring3",
            "pinky1", "pinky2", "pinky3",
        };

        static readonly string[] BoneNamesLeft = {
            "lHand",
            "lThumb1", "lThumb2", "lThumb3",
            "lIndex1", "lIndex2", "lIndex3",
            "lMid1", "lMid2", "lMid3",
            "lRing1", "lRing2", "lRing3",
            "lPinky1", "lPinky2", "lPinky3",
        };

        static readonly string[] BoneNamesRight = {
            "rHand",
            "rThumb1", "rThumb2", "rThumb3",
            "rIndex1", "rIndex2", "rIndex3",
            "rMid1", "rMid2", "rMid3",
            "rRing1", "rRing2", "rRing3",
            "rPinky1", "rPinky2", "rPinky3",
        };

        // 数据超时：超过这个时间没收到包就慢慢回到原始姿势
        const float PACKET_TIMEOUT = 0.6f;

        UdpClient _udp;
        Thread _rxThread;
        volatile bool _running;

        readonly ConcurrentQueue<float[]> _queue = new ConcurrentQueue<float[]>();

        Transform[] _bones;
        Quaternion[] _restRotation;
        Quaternion[] _applied;
        float[] _latest;
        float _lastPacketTime = -999f;
        float _currentStrength;

        long _rxCount;
        long _badCount;
        float _lastReport = -999f;
        long _lastReportCount;

        JSONStorableBool _enabledJSON;
        JSONStorableFloat _strengthJSON;
        JSONStorableBool _rightHandJSON;
        JSONStorableFloat _offsetXJSON;
        JSONStorableFloat _offsetYJSON;
        JSONStorableFloat _offsetZJSON;
        JSONStorableFloat _smoothJSON;

        public override void Init()
        {
            try
            {
                _enabledJSON = new JSONStorableBool("enabled", true);
                RegisterBool(_enabledJSON);
                CreateToggle(_enabledJSON);

                _rightHandJSON = new JSONStorableBool("rightHand", true);
                RegisterBool(_rightHandJSON);
                CreateToggle(_rightHandJSON);

                _strengthJSON = new JSONStorableFloat("strength", 1.0f, 0f, 1.5f, false);
                RegisterFloat(_strengthJSON);
                CreateSlider(_strengthJSON);

                // 骨骼朝向补偿，按欧拉角调，比自己算四元数直观
                _offsetXJSON = new JSONStorableFloat("offsetX", 0f, -180f, 180f, true);
                _offsetYJSON = new JSONStorableFloat("offsetY", 90f, -180f, 180f, true);
                _offsetZJSON = new JSONStorableFloat("offsetZ", 0f, -180f, 180f, true);
                RegisterFloat(_offsetXJSON);
                RegisterFloat(_offsetYJSON);
                RegisterFloat(_offsetZJSON);
                CreateSlider(_offsetXJSON);
                CreateSlider(_offsetYJSON);
                CreateSlider(_offsetZJSON);

                // 关节平滑，压 IMU 噪声，0 表示不平滑
                _smoothJSON = new JSONStorableFloat("smooth", 0.35f, 0f, 0.95f, false);
                RegisterFloat(_smoothJSON);
                CreateSlider(_smoothJSON);

                _lastReport = Time.realtimeSinceStartup;

                FindBones();
                StartReceiver();

                SuperController.LogMessage("[EIGlove] 初始化完成，监听 UDP " + UDP_PORT);
            }
            catch (Exception e)
            {
                SuperController.LogError("[EIGlove] 初始化失败: " + e);
            }
        }

        void FindBones()
        {
            var names = (_rightHandJSON != null && _rightHandJSON.val) ? BoneNamesRight : BoneNamesLeft;

            _bones = new Transform[JointCount];
            _restRotation = new Quaternion[JointCount];
            _applied = new Quaternion[JointCount];

            var all = containingAtom.GetComponentsInChildren<Transform>(true);
            var map = new Dictionary<string, Transform>();
            var handLike = new List<string>();

            foreach (var t in all)
            {
                if (!map.ContainsKey(t.name))
                {
                    map[t.name] = t;
                }
                var n = t.name;
                if (n.Contains("Hand") || n.Contains("Thumb") || n.Contains("Index")
                    || n.Contains("Mid") || n.Contains("Ring") || n.Contains("Pinky"))
                {
                    if (!handLike.Contains(n) && handLike.Count < 40)
                    {
                        handLike.Add(n);
                    }
                }
            }

            int found = 0;
            for (int i = 0; i < JointCount; i++)
            {
                Transform t;
                if (map.TryGetValue(names[i], out t))
                {
                    _bones[i] = t;
                    _restRotation[i] = t.localRotation;
                    _applied[i] = t.localRotation;
                    found++;
                }
            }

            SuperController.LogMessage($"[EIGlove] 找到 {found}/{JointCount} 个骨骼（{(names == BoneNamesRight ? "右手" : "左手")}）");

            if (found == 0)
            {
                // 名字对不上时把手部相关骨骼名打出来，方便照着改常量
                SuperController.LogError("[EIGlove] 一个都没匹配上，下面是人物的手部骨骼名，照着改 BoneNames：");
                for (int i = 0; i < handLike.Count; i++)
                {
                    SuperController.LogError("  " + handLike[i]);
                }
            }
        }

        void StartReceiver()
        {
            _udp = new UdpClient(UDP_PORT);
            _running = true;

            _rxThread = new Thread(() =>
            {
                var ep = new IPEndPoint(IPAddress.Any, 0);
                while (_running)
                {
                    try
                    {
                        var data = _udp.Receive(ref ep);
                        float[] q;
                        if (!EIProtocol.TryParse(data, out q))
                        {
                            _badCount++;
                            continue;
                        }

                        _queue.Enqueue(q);
                        while (_queue.Count > 2)
                        {
                            float[] drop;
                            _queue.TryDequeue(out drop);
                        }
                        _rxCount++;
                    }
                    catch (SocketException) { }
                    catch (ObjectDisposedException) { return; }
                }
            });
            _rxThread.IsBackground = true;
            _rxThread.Start();
        }

        void LateUpdate()
        {
            if (_enabledJSON == null || _bones == null)
            {
                return;
            }

            float[] incoming;
            while (_queue.TryDequeue(out incoming))
            {
                _latest = incoming;
                _lastPacketTime = Time.realtimeSinceStartup;
            }

            bool fresh = (Time.realtimeSinceStartup - _lastPacketTime) < PACKET_TIMEOUT;
            bool want = _enabledJSON.val && fresh;

            float targetStrength = want
                ? (_strengthJSON != null ? _strengthJSON.val : 1.0f)
                : 0.0f;

            // 强度用指数逼近，断流时手掌会平滑落回原始姿势而不是硬切
            float k = 1.0f - Mathf.Exp(-Time.deltaTime * 6.0f);
            _currentStrength = Mathf.Lerp(_currentStrength, targetStrength, k);
            float strength = _currentStrength;

            Quaternion mount = Quaternion.Euler(
                _offsetXJSON != null ? _offsetXJSON.val : 0f,
                _offsetYJSON != null ? _offsetYJSON.val : 90f,
                _offsetZJSON != null ? _offsetZJSON.val : 0f);

            float smooth = _smoothJSON != null ? _smoothJSON.val : 0.35f;
            float slerpK = 1.0f - smooth;

            for (int i = 0; i < JointCount; i++)
            {
                var bone = _bones[i];
                if (bone == null)
                {
                    continue;
                }

                Quaternion goal;
                if (want && _latest != null)
                {
                    int o = i * 4;
                    var q = new Quaternion(_latest[o + 1], _latest[o + 2], _latest[o + 3], _latest[o]);
                    goal = mount * q;
                }
                else
                {
                    goal = _restRotation[i];
                }

                // 目标先按强度插值，再按平滑系数逼近，避免抖动
                Quaternion blended = Quaternion.Slerp(_restRotation[i], goal, strength);
                _applied[i] = Quaternion.Slerp(_applied[i], blended, slerpK);
                bone.localRotation = _applied[i];
            }

            Report();
        }

        void Report()
        {
            float now = Time.realtimeSinceStartup;
            if (now - _lastReport < 5f)
            {
                return;
            }

            long delta = _rxCount - _lastReportCount;
            float fps = (now - _lastReport > 0.5f) ? delta / (now - _lastReport) : 0f;

            if (_rxCount == 0)
            {
                SuperController.LogMessage($"[EIGlove] 还没收到数据，检查上位机是否在向 {UDP_PORT} 端口发包（坏包 {_badCount}）");
            }
            else
            {
                SuperController.LogMessage($"[EIGlove] 收包 {fps:F0}/s，累计 {_rxCount}，坏包 {_badCount}");
            }

            _lastReport = now;
            _lastReportCount = _rxCount;
        }

        void OnDestroy()
        {
            _running = false;
            try { if (_udp != null) _udp.Close(); } catch { }
            try
            {
                if (_rxThread != null && _rxThread.IsAlive)
                {
                    _rxThread.Join(200);
                }
            }
            catch { }
        }
    }

    // 协议解析。与 vam-plugin/EIGloveProtocol.cs 是同一份逻辑
    static class EIProtocol
    {
        public const int PacketSize = 272;
        public const int JointCount = 16;

        public static bool TryParse(byte[] data, out float[] quaternions)
        {
            quaternions = null;
            if (data == null || data.Length != PacketSize)
            {
                return false;
            }
            if (data[0] != (byte)'E' || data[1] != (byte)'I'
                || data[2] != (byte)'G' || data[3] != (byte)'V')
            {
                return false;
            }
            ushort version = (ushort)(data[4] | (data[5] << 8));
            if (version != 1)
            {
                return false;
            }

            var outQ = new float[JointCount * 4];
            for (int i = 0; i < JointCount; i++)
            {
                int o = 16 + i * 16;
                outQ[i * 4 + 0] = BitConverter.ToSingle(data, o);
                outQ[i * 4 + 1] = BitConverter.ToSingle(data, o + 4);
                outQ[i * 4 + 2] = BitConverter.ToSingle(data, o + 8);
                outQ[i * 4 + 3] = BitConverter.ToSingle(data, o + 12);
            }
            quaternions = outQ;
            return true;
        }
    }
}
