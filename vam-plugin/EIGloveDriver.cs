// EIGloveDriver —— 把 EI Controller 手套的数据套到 VaM 的人体骨骼上
//
// 安装：放到 VaM 的 Custom/Scripts 目录，在场景里 Add Plugin 选它。
// 数据来源见 docs/vam-协议.md，上位机用 tools/eic_bridge.py。
//
// 协议解析逻辑在本文件底部（EIProtocol 类）。同样一份逻辑在
// EIGloveProtocol.cs 里有一个独立副本，用于 Linux 上跑单元测试。
// 改解析规则时两处要同步。

using System;
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

        readonly Queue<float[]> _queue = new Queue<float[]>();
        readonly object _queueLock = new object();

        Transform[] _bones;
        Quaternion[] _restRotation;
        Quaternion[] _applied;
        float[] _latest;
        float _lastPacketTime = -999f;
        float _currentStrength;

        // VaM 1.22 的人物骨骼是 DAZBone（挂在骨骼 Transform 上），皮肤由
        // DAZSkinV2 做 GPU 蒙皮。名字匹配到同名 Transform 不一定是皮肤骨骼，
        // 所以优先通过 DAZCharacter.rootBonesForSkinning 解析。
        DAZBones _dazBones;
        string[] _activeNames;
        bool _renderHookInstalled;
        float _lastOverrideCheck = -999f;
        // 只有正在跟随数据（或回位动画还没走完）时才写骨骼。
        // 断流后完全放手，避免每帧强写 rest 与 VaM 的 idle 动画打架。
        bool _writing;
        long _preRenderCount;
        float _lastPreDiff = -1f;

        // 官方手部输出通道（RightHandControl），测试版先只驱动右手食指三节
        MeshVR.Hands.HandOutput _handOut;
        bool _handOutReady;
        bool _outputConnectedWas;

        // 右手关节弹簧（MeshedVR 的 AdjustJointSprings），百分太低时 targetRotation 拉不动骨骼
        HandControl _handControl;
        float _springWasPercent;
        bool _springWasOn;
        // 五指三节上一帧的 bend，用于单帧限幅
        readonly float[] _lastBend = new float[15];

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
        JSONStorableFloat _fingerMapModeJSON;

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
                _smoothJSON = new JSONStorableFloat("smooth", 0.7f, 0f, 0.95f, false);
                RegisterFloat(_smoothJSON);
                CreateSlider(_smoothJSON);

                // 手指轴向映射模式（0~5），运行时拖动切换，不用重载；实测 3 = (x, z, -y) 正确
                _fingerMapModeJSON = new JSONStorableFloat("fingerMap", 3f, 0f, 5f, true);
                RegisterFloat(_fingerMapModeJSON);
                CreateSlider(_fingerMapModeJSON);

                _lastReport = Time.realtimeSinceStartup;

                FindBones();
                FindHandOutput();
                FindHandControl();
                DiagProbe();
                DiagHands();

                // 渲染前再写一次骨骼。VaM 的 LateUpdate 里还有自己的骨骼/蒙皮
                // 更新，普通 LateUpdate 可能被它覆盖；onPreRender 在所有
                // LateUpdate 之后再执行，能保证渲染用的是我们的姿态。
                Camera.onPreRender += OnPreRenderHook;
                _renderHookInstalled = true;

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
            _activeNames = names;

            _bones = new Transform[JointCount];
            _restRotation = new Quaternion[JointCount];
            _applied = new Quaternion[JointCount];

            try
            {
                var dc = containingAtom.GetComponentInChildren<DAZCharacter>(true);
                if (dc != null)
                {
                    _dazBones = dc.rootBonesForSkinning;
                }
            }
            catch (Exception e)
            {
                SuperController.LogError("[EIGlove] 取 DAZCharacter 失败: " + e.Message);
            }

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
            int foundViaDaz = 0;
            for (int i = 0; i < JointCount; i++)
            {
                Transform t = null;
                DAZBone db = null;
                if (_dazBones != null)
                {
                    try
                    {
                        db = _dazBones.GetDAZBone(names[i]);
                    }
                    catch { }
                }

                if (db != null)
                {
                    t = db.transform;
                    foundViaDaz++;
                }
                else
                {
                    Transform tmp;
                    if (map.TryGetValue(names[i], out tmp))
                    {
                        t = tmp;
                    }
                }

                if (t != null)
                {
                    _bones[i] = t;
                    _restRotation[i] = t.localRotation;
                    _applied[i] = t.localRotation;
                    found++;
                }
            }

            SuperController.LogMessage($"[EIGlove] 找到 {found}/{JointCount} 个骨骼（{(names == BoneNamesRight ? "右手" : "左手")}，其中 {foundViaDaz} 个经 DAZBone 解析）");

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

        // 诊断 VaM 官方手部管线：HandInput → HandOutput → FingerOutput
        void DiagHands()
        {
            try
            {
                var inputs = containingAtom.GetComponentsInChildren<MeshVR.Hands.HandInput>(true);
                SuperController.LogMessage($"[EIGlove:DIAG] HandInput 数量 {inputs.Length}");
                for (int i = 0; i < inputs.Length; i++)
                {
                    var hi = inputs[i];
                    if (hi == null)
                    {
                        continue;
                    }
                    string b = "n/a";
                    if (hi.indexProximal != null)
                    {
                        b = $"{hi.indexProximal.currentBend:F3}/{hi.indexProximal.currentSpread:F3}/{hi.indexProximal.currentTwist:F3}";
                    }
                    SuperController.LogMessage($"[EIGlove:DIAG] HandInput[{i}] path={TransformPath(hi.transform)} indexProximal(bend/spread/twist)={b}");
                }

                var outputs = containingAtom.GetComponentsInChildren<MeshVR.Hands.HandOutput>(true);
                SuperController.LogMessage($"[EIGlove:DIAG] HandOutput 数量 {outputs.Length}");
                for (int i = 0; i < outputs.Length; i++)
                {
                    var ho = outputs[i];
                    if (ho == null)
                    {
                        continue;
                    }
                    string vals = "n/a";
                    if (ho.indexProximalBendJSON != null)
                    {
                        vals = $"i1b={ho.indexProximalBendJSON.val:F3} i1s={ho.indexProximalSpreadJSON.val:F3} i1t={ho.indexProximalTwistJSON.val:F3} i2b={ho.indexMiddleBendJSON.val:F3} i3b={ho.indexDistalBendJSON.val:F3}";
                    }
                    string conn = "n/a";
                    if (ho.inputConnectedJSON != null)
                    {
                        conn = $"input={ho.inputConnectedJSON.val} output={(ho.outputConnectedJSON != null ? ho.outputConnectedJSON.val.ToString() : "?")}";
                    }
                    SuperController.LogMessage($"[EIGlove:DIAG] HandOutput[{i}] path={TransformPath(ho.transform)} {vals} {conn}");
                }

                var fingers = containingAtom.GetComponentsInChildren<MeshVR.Hands.FingerOutput>(true);
                SuperController.LogMessage($"[EIGlove:DIAG] FingerOutput 数量 {fingers.Length}");
                if (fingers.Length > 0 && fingers[0] != null)
                {
                    var fo = fingers[0];
                    SuperController.LogMessage($"[EIGlove:DIAG] FingerOutput[0] path={TransformPath(fo.transform)} bend={fo.currentBend:F3} spread={fo.currentSpread:F3} twist={fo.currentTwist:F3} bendEnabled={fo.bendEnabled}");
                }

                // 食指三节的关节状态：joint 是否为空、旋转轴是否被锁、targetRotation 是否在变
                for (int i = 0; i < fingers.Length; i++)
                {
                    var fo = fingers[i];
                    if (fo == null)
                    {
                        continue;
                    }
                    string n = fo.transform.name;
                    if (n != "rIndex1" && n != "rIndex2" && n != "rIndex3")
                    {
                        continue;
                    }
                    var cj = fo.GetComponent<ConfigurableJoint>();
                    string j = "null";
                    if (cj != null)
                    {
                        var tr = cj.targetRotation;
                        j = $"motion=({cj.angularXMotion},{cj.angularYMotion},{cj.angularZMotion}) target=({tr.x:F3},{tr.y:F3},{tr.z:F3},{tr.w:F3})";
                    }
                    SuperController.LogMessage($"[EIGlove:DIAG] {n} currentBend={fo.currentBend:F2} joint={j}");
                }
            }
            catch (Exception e)
            {
                SuperController.LogError("[EIGlove:DIAG] DiagHands 失败: " + e);
            }
        }

        static string TransformPath(Transform t)
        {
            string s = t.name;
            int guard = 0;
            for (var p = t.parent; p != null && guard < 8; p = p.parent)
            {
                s = p.name + "/" + s;
                guard++;
            }
            return s;
        }

        // 找官方手部输出通道（RightHandControl / LeftHandControl）
        void FindHandOutput()
        {
            try
            {
                var outs = containingAtom.GetComponentsInChildren<MeshVR.Hands.HandOutput>(true);
                for (int i = 0; i < outs.Length; i++)
                {
                    var ho = outs[i];
                    if (ho == null)
                    {
                        continue;
                    }
                    if (TransformPath(ho.transform).Contains("RightHandControl"))
                    {
                        _handOut = ho;
                        break;
                    }
                }

                if (_handOut != null)
                {
                    if (_handOut.outputConnectedJSON != null)
                    {
                        _outputConnectedWas = _handOut.outputConnectedJSON.val;
                        // 必须走 SetVal 才会触发 SetOutputConnected，更新内部 _outputConnected；
                        // 直接赋 .val 不会更新，导致 SetXxxBend 被静默跳过
                        _handOut.outputConnectedJSON.SetVal(true);
                    }
                    _handOutReady = true;
                    string range = "n/a";
                    if (_handOut.indexProximalBendJSON != null)
                    {
                        range = $"[{_handOut.indexProximalBendJSON.min:F1}, {_handOut.indexProximalBendJSON.max:F1}]";
                    }
                    SuperController.LogMessage($"[EIGlove] HandOutput 找到 {TransformPath(_handOut.transform)}，output 已连接，indexBend 范围={range}");
                }
                else
                {
                    SuperController.LogError("[EIGlove] 未找到 RightHandControl 的 HandOutput");
                }
            }
            catch (Exception e)
            {
                SuperController.LogError("[EIGlove] FindHandOutput 失败: " + e.Message);
            }
        }

        // 找右手 HandControl，并把关节弹簧调到最大——FingerOutput 只设 targetRotation，
        // 真正拉动骨骼的是 AdjustJointSprings.percent
        void FindHandControl()
        {
            try
            {
                var hcs = containingAtom.GetComponentsInChildren<HandControl>(true);
                SuperController.LogMessage($"[EIGlove] HandControl 数量 {hcs.Length}");
                for (int i = 0; i < hcs.Length; i++)
                {
                    var hc = hcs[i];
                    if (hc == null)
                    {
                        continue;
                    }
                    string boneName = (hc.indexProximalBone != null) ? hc.indexProximalBone.transform.name : "null";
                    string springPct = (hc.handJointSprings != null) ? hc.handJointSprings.percent.ToString("F1") : "null";
                    SuperController.LogMessage($"[EIGlove] HandControl[{i}] path={TransformPath(hc.transform)} indexProximalBone={boneName} springsPercent={springPct}");
                    if (boneName.StartsWith("r"))
                    {
                        _handControl = hc;
                    }
                }

                if (_handControl != null)
                {
                    if (_handControl.handJointSprings != null)
                    {
                        _springWasPercent = _handControl.handJointSprings.percent;
                        _springWasOn = _handControl.handJointSprings.on;
                        _handControl.handJointSprings.on = true;
                        // 100 才拉得动；全身抽改用增益和单帧限幅来压，而不是降弹簧
                        _handControl.handJointSprings.percent = 100f;
                        _handControl.handJointSprings.ResyncSprings();
                        SuperController.LogMessage($"[EIGlove] 右手关节弹簧 percent {_springWasPercent:F1} -> 100，已 Resync");
                    }
                    if (_handControl.handGraspStrengthJSON != null)
                    {
                        SuperController.LogMessage($"[EIGlove] 右手握力 {_handControl.handGraspStrengthJSON.val:F2}");
                    }
                }
                else
                {
                    SuperController.LogError("[EIGlove] 未找到右手的 HandControl");
                }
            }
            catch (Exception e)
            {
                SuperController.LogError("[EIGlove] FindHandControl 失败: " + e.Message);
            }
        }

        // 一次性诊断：确认 _bones[5]（index2）到底是什么节点，是否参与蒙皮
        void DiagProbe()
        {
            try
            {
                var all = containingAtom.GetComponentsInChildren<Transform>(true);
                int sameName = 0;
                foreach (var t in all)
                {
                    if (t.name == _activeNames[5])
                    {
                        sameName++;
                    }
                }

                var probe = _bones[5];
                if (probe == null)
                {
                    SuperController.LogMessage("[EIGlove:DIAG] probe null（i2 没找到）");
                    return;
                }

                var kids = new List<string>();
                for (int i = 0; i < probe.childCount && i < 10; i++)
                {
                    kids.Add(probe.GetChild(i).name);
                }

                DAZBone db = null;
                if (_dazBones != null)
                {
                    try
                    {
                        db = _dazBones.GetDAZBone(_activeNames[5]);
                    }
                    catch { }
                }

                string ctl = "none";
                if (db != null && db.control != null)
                {
                    ctl = db.control.name;
                }

                string skinInfo = "none";
                bool inNodes = false;
                var skins = containingAtom.GetComponentsInChildren<DAZSkinV2>(true);
                if (skins.Length > 0)
                {
                    int nodeCount = 0;
                    bool rootOK = false;
                    bool rootMatch = false;
                    bool delay1 = false;
                    bool multithread = false;
                    foreach (var sk in skins)
                    {
                        if (sk == null)
                        {
                            continue;
                        }
                        if (sk.root != null)
                        {
                            rootOK = true;
                            if (sk.root == _dazBones)
                            {
                                rootMatch = true;
                            }
                        }
                        delay1 = delay1 || sk.delayDisplayOneFrame;
                        multithread = multithread || sk.useMultithreadedSkinning;
                        if (sk.nodes != null)
                        {
                            nodeCount += sk.nodes.Length;
                            foreach (var nd in sk.nodes)
                            {
                                if (nd != null && nd.name == probe.name)
                                {
                                    inNodes = true;
                                }
                            }
                        }
                    }
                    skinInfo = $"skins={skins.Length} nodes={nodeCount} rootOK={rootOK} rootMatch={rootMatch} multithread={multithread} delay1={delay1}";
                }

                SuperController.LogMessage(
                    $"[EIGlove:DIAG] i2 name={probe.name} path={TransformPath(probe)} children={probe.childCount} [{string.Join(",", kids.ToArray())}] sameName={sameName} dazBone={(db != null)} jointDisabled={(db != null ? db.jointRotationDisabled.ToString() : "n/a")} control={ctl} inSkinNodes={inNodes} {skinInfo}");

                if (!inNodes)
                {
                    // 骨骼不在蒙皮节点里，打印 skin 里真正的手部节点名，方便对照
                    var cand = new List<string>();
                    foreach (var sk in skins)
                    {
                        if (sk == null || sk.nodes == null)
                        {
                            continue;
                        }
                        foreach (var nd in sk.nodes)
                        {
                            if (nd != null && nd.name != null
                                && (nd.name.Contains("Index") || nd.name.Contains("Hand") || nd.name.Contains("Thumb"))
                                && !cand.Contains(nd.name))
                            {
                                cand.Add(nd.name);
                            }
                        }
                    }
                    var shown = new List<string>();
                    for (int i = 0; i < cand.Count && i < 20; i++)
                    {
                        shown.Add(cand[i]);
                    }
                    SuperController.LogMessage("[EIGlove:DIAG] skin 里的手部节点: " + string.Join(",", shown.ToArray()));
                }
            }
            catch (Exception e)
            {
                SuperController.LogError("[EIGlove:DIAG] probe 失败: " + e);
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

                        lock (_queueLock)
                        {
                            _queue.Enqueue(q);
                            while (_queue.Count > 2)
                            {
                                _queue.Dequeue();
                            }
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

            // 把我们的 onPreRender 回调重新挂到 delegate 链末尾，
            // 确保在 VaM 自己的 onPreRender 之后执行（它也会注册这个事件）。
            if (_renderHookInstalled)
            {
                Camera.onPreRender -= OnPreRenderHook;
                Camera.onPreRender += OnPreRenderHook;
            }

            lock (_queueLock)
            {
                while (_queue.Count > 0)
                {
                    _latest = _queue.Dequeue();
                    _lastPacketTime = Time.realtimeSinceStartup;
                }
            }

            bool fresh = (Time.realtimeSinceStartup - _lastPacketTime) < PACKET_TIMEOUT;
            bool want = _enabledJSON.val && fresh;

            float targetStrength = want
                ? (_strengthJSON != null ? _strengthJSON.val : 1.0f)
                : 0.0f;

            // 直通模式：立刻用满强度；否则按指数逼近（断流时手慢慢回位）
            float strength;
            if (DIRECT)
            {
                strength = targetStrength;
                _currentStrength = targetStrength;
            }
            else
            {
                float k = 1.0f - Mathf.Exp(-Time.deltaTime * 6.0f);
                _currentStrength = Mathf.Lerp(_currentStrength, targetStrength, k);
                strength = _currentStrength;
            }

            // 强度超过 1 会让 Slerp 外插，产生过冲/抖动，这里钳死
            strength = Mathf.Clamp(strength, 0f, 1f);

            Quaternion mount = Quaternion.Euler(
                _offsetXJSON != null ? _offsetXJSON.val : 0f,
                _offsetYJSON != null ? _offsetYJSON.val : 90f,
                _offsetZJSON != null ? _offsetZJSON.val : 0f);

            float smooth = _smoothJSON != null ? _smoothJSON.val : 0.35f;
            float slerpK = DIRECT ? 1.0f : (1.0f - smooth);

            // 断流且回位动画结束后就停止写骨骼，把控制权还给 VaM
            _writing = want || _currentStrength > 0.01f;

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
                // HANDOUT_TEST 下手指出 HandOutput 驱动，但 hand 骨骼仍要写，手背才能跟着转
                if (_writing && (!HANDOUT_TEST || i == 0))
                {
                    bone.localRotation = BoneToUnity(i, _applied[i], (_fingerMapModeJSON != null) ? (int)_fingerMapModeJSON.val : 1);
                }
            }

            // 直接写死测试：绕过数据/限幅，确认 HandOutput 通道本身（写近节 Proximal）
            if (HANDOUT_TEST && TEST_DIRECT && _handOutReady && _handOut != null)
            {
                _handOut.indexProximalBendJSON.val = 30f;
                if (DIAG && (Time.frameCount % 30 == 0))
                {
                    float cur = (_handOut.indexProximal != null) ? _handOut.indexProximal.currentBend : -999f;
                    float springs = (_handControl != null && _handControl.handJointSprings != null)
                        ? _handControl.handJointSprings.percent : -1f;
                    SuperController.LogMessage($"[EIGlove:DIAG] direct i1b={_handOut.indexProximalBendJSON.val:F1} cur={cur:F1} springs={springs:F1}");
                }
            }
            else if (HANDOUT_TEST && _handOutReady && _handOut != null && _applied != null)
            {
                WriteFingerBend(_applied[1], _handOut.thumbProximalBendJSON, 0);
                WriteFingerBend(_applied[2], _handOut.thumbMiddleBendJSON, 1);
                WriteFingerBend(_applied[3], _handOut.thumbDistalBendJSON, 2);
                WriteFingerBend(_applied[4], _handOut.indexProximalBendJSON, 3);
                WriteFingerBend(_applied[5], _handOut.indexMiddleBendJSON, 4);
                WriteFingerBend(_applied[6], _handOut.indexDistalBendJSON, 5);
                WriteFingerBend(_applied[7], _handOut.middleProximalBendJSON, 6);
                WriteFingerBend(_applied[8], _handOut.middleMiddleBendJSON, 7);
                WriteFingerBend(_applied[9], _handOut.middleDistalBendJSON, 8);
                WriteFingerBend(_applied[10], _handOut.ringProximalBendJSON, 9);
                WriteFingerBend(_applied[11], _handOut.ringMiddleBendJSON, 10);
                WriteFingerBend(_applied[12], _handOut.ringDistalBendJSON, 11);
                WriteFingerBend(_applied[13], _handOut.pinkyProximalBendJSON, 12);
                WriteFingerBend(_applied[14], _handOut.pinkyMiddleBendJSON, 13);
                WriteFingerBend(_applied[15], _handOut.pinkyDistalBendJSON, 14);
            }

            // 每帧打日志会拖慢 VaM（180fps 时磁盘/面板会被刷爆），限到约 6Hz
            if (DIAG && _latest != null && _bones[0] != null && (Time.frameCount % 15 == 0))
            {
                string msg = string.Format(
                    "[EIGlove:DIAG] en={0} st={1:F2} cur={2:F2} sm={3:F2} fr={4} | hand q=({5:F2},{6:F2},{7:F2},{8:F2}) i2=({9:F2},{10:F2},{11:F2},{12:F2}) | i2世界 y={13:F3} z={14:F3}",
                    _enabledJSON.val,
                    _strengthJSON != null ? _strengthJSON.val : -1f,
                    strength,
                    smooth,
                    fresh,
                    _latest[0], _latest[1], _latest[2], _latest[3],
                    _latest[20], _latest[21], _latest[22], _latest[23],
                    _bones[5].position.y,
                    _bones[5].position.z
                );
                SuperController.LogMessage(msg);
            }

            Report();
        }

        // 诊断：打印实际应用到骨骼的角度
        const bool DIAG = true;
        // 直通模式：数据来了立刻满强度、不平滑。调试手感用，调好后改回 false 恢复平滑。
        // 真实手套/合成数据的 IMU 噪声在直通模式下会表现为高频抖动，默认关闭。
        const bool DIRECT = false;
        // 渲染前重写骨骼：绕开 VaM 自己的 LateUpdate 骨骼/蒙皮更新
        const bool RENDER_WRITE = true;
        // 动捕手柄场景：物理冻结，直接写骨骼（冻结时物理不覆盖）。
        // 物理开启时 HandOutput 的 targetRotation 才会被跟随，但会被物理覆盖，不适合长时间动捕。
        const bool HANDOUT_TEST = false;
        // 直接写死测试：绕过数据计算，确认 HandOutput→骨骼通道本身是否活着
        const bool TEST_DIRECT = false;
        // 四元数 → bend 的放大系数（0.8 太小，工具大幅弯曲手指才动一点）
        const float BEND_GAIN = 2.0f;
        // 单帧 bend 最大变化量（度），压物理链冲击
        const float BEND_STEP_MAX = 5f;

        // 覆盖检测：Update 里读回 i2 的 localRotation，和上一帧我们写入的值比。
        // 如果差了，说明 VaM 的其它系统（控制器/物理/蒙皮）在 LateUpdate 之后又写了它。
        void Update()
        {
            if (!DIAG || _bones == null || _bones[5] == null || _latest == null)
            {
                return;
            }

            float now = Time.realtimeSinceStartup;
            if (now - _lastOverrideCheck < 2f)
            {
                return;
            }
            _lastOverrideCheck = now;

            float diff = Quaternion.Angle(_bones[5].localRotation, _applied[5]);

            string dazAngles = "n/a";
            if (_dazBones != null)
            {
                try
                {
                    var db = _dazBones.GetDAZBone(_activeNames[5]);
                    if (db != null)
                    {
                        var a = db.GetAnglesDegrees();
                        dazAngles = $"{a.x:F1},{a.y:F1},{a.z:F1}";
                    }
                }
                catch { }
            }

            if (diff > 1f)
            {
                SuperController.LogMessage($"[EIGlove:DIAG] 覆盖检测: lIndex2 当前与写入相差 {diff:F1} 度（LateUpdate 之后有其它系统在写它），DAZBone角度=({dazAngles}) | onPreRender {_preRenderCount} 次，写后差 {_lastPreDiff:F1} 度");
            }
            else
            {
                SuperController.LogMessage($"[EIGlove:DIAG] 覆盖检测: lIndex2 与写入一致（差 {diff:F1} 度），DAZBone角度=({dazAngles}) | onPreRender {_preRenderCount} 次，写后差 {_lastPreDiff:F1} 度");
            }
        }

        void OnPreRenderHook(Camera cam)
        {
            if (!RENDER_WRITE || cam == null || cam.cameraType != CameraType.Game)
            {
                return;
            }
            WriteBonesNow();
            _preRenderCount++;
            if (_bones != null && _bones[5] != null && _applied != null)
            {
                _lastPreDiff = Quaternion.Angle(_bones[5].localRotation, _applied[5]);
            }
        }

        // 用户标定：VaM z = 固件 y、VaM y = 固件 x 已对；
        // 另外两组（x/y 的源）互换：VaM x = 固件 x，VaM y = 固件 z
        // 工具轴 → VaM 骨骼局部轴（fingerMap 滑块运行时切换，0~5）
        static Quaternion BoneToUnity(int i, Quaternion q, int mode)
        {
            if (i == 0)
            {
                return new Quaternion(q.x, q.z, q.y, q.w);
            }
            switch (mode)
            {
                case 0: return new Quaternion(q.x, q.z, q.y, q.w);
                case 1: return new Quaternion(-q.x, q.z, q.y, q.w);
                case 2: return new Quaternion(q.x, -q.z, q.y, q.w);
                case 3: return new Quaternion(q.x, q.z, -q.y, q.w);
                case 4: return new Quaternion(-q.x, -q.z, q.y, q.w);
                case 5: return new Quaternion(q.z, q.x, q.y, q.w);
                default: return q;
            }
        }

        void WriteBonesNow()
        {
            if (_bones == null || _applied == null)
            {
                return;
            }
            for (int i = 0; i < JointCount; i++)
            {
                // HANDOUT_TEST 下只重写 hand，手指交给 HandOutput，避免双驱动
                if (HANDOUT_TEST && i != 0)
                {
                    continue;
                }
                if (_bones[i] != null)
                {
                    _bones[i].localRotation = BoneToUnity(i, _applied[i], (_fingerMapModeJSON != null) ? (int)_fingerMapModeJSON.val : 1);
                }
            }
        }

        // 从最近一包数据里取某个关节的四元数（协议顺序 w,x,y,z）
        Quaternion DataQ(int joint)
        {
            int o = joint * 4;
            return new Quaternion(_latest[o + 1], _latest[o + 2], _latest[o + 3], _latest[o]);
        }

        // 把相对四元数转成 HandOutput 的 bend（角度），带放大系数和单帧限幅
        void WriteFingerBend(Quaternion q, JSONStorableFloat json, int seg)
        {
            if (json == null || seg < 0 || seg >= _lastBend.Length)
            {
                return;
            }
            // 数据绕 x 轴（工具输入 X=30 → 四元数 x 分量），直接取 X 欧拉角
            float x = q.eulerAngles.x;
            if (x > 180f)
            {
                x -= 360f;
            }
            // mount 把数据的 x 旋转分散到 x/y，Z 恒为 0；改取 X 欧拉角
            float ex = q.eulerAngles.x;
            float ey = q.eulerAngles.y;
            float ez = q.eulerAngles.z;
            if (ex > 180f)
            {
                ex -= 360f;
            }
            float bend = ex * BEND_GAIN;

            if (DIAG && seg == 4 && (Time.frameCount % 30 == 0))
            {
                SuperController.LogMessage($"[EIGlove:DIAG] bend seg={seg} euler=({ex:F1},{ey:F1},{ez:F1}) bend={bend:F1}");
            }

            // 关节极限之外的幅度直接放弃，避免物理把手指折崩
            bend = Mathf.Clamp(bend, -50f, 50f);

            // 单帧限幅：目标变化再快也只让它每帧走 BEND_STEP_MAX 度，
            // 避免 targetRotation 突变把力传给整条物理链
            float last = _lastBend[seg];
            if (bend > last + BEND_STEP_MAX)
            {
                bend = last + BEND_STEP_MAX;
            }
            else if (bend < last - BEND_STEP_MAX)
            {
                bend = last - BEND_STEP_MAX;
            }
            _lastBend[seg] = bend;

            if (bend < json.min)
            {
                bend = json.min;
            }
            if (bend > json.max)
            {
                bend = json.max;
            }
            json.val = bend;
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
            if (_renderHookInstalled)
            {
                try { Camera.onPreRender -= OnPreRenderHook; } catch { }
                _renderHookInstalled = false;
            }
            _running = false;
            if (_handOut != null && _handOut.outputConnectedJSON != null)
            {
                try { _handOut.outputConnectedJSON.val = _outputConnectedWas; } catch { }
            }
            if (_handControl != null && _handControl.handJointSprings != null)
            {
                try
                {
                    _handControl.handJointSprings.percent = _springWasPercent;
                    _handControl.handJointSprings.on = _springWasOn;
                }
                catch { }
            }
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
