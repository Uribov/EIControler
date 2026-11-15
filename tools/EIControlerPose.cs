// EIControlerPose —— 手部姿态编辑器（3D）
//
// 坐标标定和固件一致：x+ 指向肘部，y+ 指向身体右侧，z+ 指向天空。
// 圆柱指节 + 球关节的手部模型；拖动任意指节调整该关节的局部旋转，
// 拖动右下角三轴控制球调整整只手；右键拖动转视角。200Hz 按 EIGV 协议发送。
//
// 编译（系统自带 .NET Framework 编译器）：
//   csc /target:winexe /optimize+ /r:System.Windows.Forms.dll /r:System.Drawing.dll
//       /r:System.Numerics.dll /out:EIControlerPose.exe EIControlerPose.cs

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Windows.Forms;
using SN = System.Numerics;

static class Program
{
    [STAThread]
    static void Main(string[] args)
    {
        string host = "127.0.0.1";
        int port = 9001;
        int hand = 1;
        for (int i = 0; i < args.Length; i++)
        {
            if (args[i] == "--host" && i + 1 < args.Length) host = args[++i];
            else if (args[i] == "--udp-port" && i + 1 < args.Length) int.TryParse(args[++i], out port);
            else if (args[i] == "--hand" && i + 1 < args.Length) int.TryParse(args[++i], out hand);
        }
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        Application.Run(new PoseForm(host, port, hand));
    }
}

class PoseForm : Form
{
    const string MAGIC = "EIGV";
    const ushort VERSION = 1;
    const double SEND_HZ = 200.0;

    static readonly SN.Vector3 ELBOW = new SN.Vector3(1, 0, 0);
    static readonly SN.Vector3 UP = new SN.Vector3(0, 0, 1);

    class Bone
    {
        public string Name;
        public int Parent;
        public SN.Vector3 Offset;
        public SN.Vector3 Axis;
        public float Length;
        public float Width;
        public SN.Vector3 BoxCenter;  // 手掌长方体中心（相对 hand）
        public SN.Vector3 BoxHalf;    // 非零时画扁长方体（手掌）
        public SN.Quaternion Rot = SN.Quaternion.Identity;
    }

    readonly Bone[] bones = new Bone[16];
    readonly string[] names = {
        "hand", "thumb1","thumb2","thumb3", "index1","index2","index3",
        "middle1","middle2","middle3", "ring1","ring2","ring3", "pinky1","pinky2","pinky3"
    };
    static readonly Color[] fingerColors = {
        Color.FromArgb(255, 120, 120), // hand
        Color.FromArgb(255, 160, 90),  // thumb
        Color.FromArgb(255, 200, 80),  // index
        Color.FromArgb(150, 220, 90),  // middle
        Color.FromArgb(80, 200, 200),  // ring
        Color.FromArgb(120, 150, 255), // pinky
    };

    SN.Quaternion handRot = SN.Quaternion.Identity;
    int dragBone = -1;
    int ringAxis = -1;
    float ringLastAngle;
    Point dragLast;
    float yaw = 0f, pitch = 1.45f;
    float distance = 2.0f;

    readonly string host;
    readonly int port;
    readonly int hand;
    volatile bool running = true;
    Thread sender;
    long sent;
    float rate;
    DateTime lastRate = DateTime.Now;
    long lastCount;

    readonly Font font = new Font("Microsoft YaHei UI", 10f);
    readonly Font small = new Font("Microsoft YaHei UI", 8.5f);
    readonly System.Windows.Forms.Timer timer;

    ComboBox targetCombo;
    TextBox[] angleBox;
    Button applyBtn;

    public PoseForm(string host, int port, int hand)
    {
        this.host = host;
        this.port = port;
        this.hand = hand;

        Text = "EIGlove 手部姿态编辑器（3D） · x+肘 / y+右 / z+天（同固件机体系）";
        ClientSize = new Size(1000, 920);
        FormBorderStyle = FormBorderStyle.FixedSingle;
        MaximizeBox = false;
        StartPosition = FormStartPosition.CenterScreen;
        BackColor = Color.FromArgb(22, 22, 28);
        DoubleBuffered = true;
        KeyPreview = true;

        InitSkeleton();
        InitTools();

        timer = new System.Windows.Forms.Timer();
        timer.Interval = 33;
        timer.Tick += delegate
        {
            DateTime now = DateTime.Now;
            if ((now - lastRate).TotalSeconds >= 0.5)
            {
                rate = (float)((sent - lastCount) / (now - lastRate).TotalSeconds);
                lastCount = sent;
                lastRate = now;
            }
            Invalidate();
        };
        timer.Start();

        sender = new Thread(SendLoop);
        sender.IsBackground = true;
        sender.Start();
    }

    // ---------- 右侧工具面板 ----------

    void InitTools()
    {
        int x0 = 790;
        Label title = new Label
        {
            Text = "关节角度（绝对欧拉角，度）",
            Left = x0, Top = 80, Width = 200,
            ForeColor = Color.White, BackColor = Color.Transparent
        };
        Controls.Add(title);

        targetCombo = new ComboBox
        {
            Left = x0, Top = 108, Width = 200,
            DropDownStyle = ComboBoxStyle.DropDownList
        };
        targetCombo.Items.Add("整手（hand）");
        for (int i = 1; i < 16; i++) targetCombo.Items.Add(names[i]);
        targetCombo.SelectedIndex = 0;
        Controls.Add(targetCombo);

        Label poseLab = new Label
        {
            Text = "动作预设",
            Left = x0, Top = 470, Width = 200, Height = 18,
            ForeColor = Color.Gainsboro, BackColor = Color.Transparent
        };
        Controls.Add(poseLab);

        ComboBox poseCombo = new ComboBox
        {
            Left = x0, Top = 490, Width = 200, Height = 24,
            DropDownStyle = ComboBoxStyle.DropDownList
        };
        poseCombo.Items.Add("自定义");
        poseCombo.Items.Add("张开（标定）");
        poseCombo.Items.Add("握拳");
        poseCombo.Items.Add("点赞");
        poseCombo.Items.Add("挥手");
        poseCombo.Items.Add("OK");
        poseCombo.Items.Add("比耶");
        poseCombo.Items.Add("竖中指");
        poseCombo.SelectedIndex = 0;
        poseCombo.SelectedIndexChanged += delegate
        {
            if (poseCombo.SelectedIndex > 0)
            {
                ApplyPreset(poseCombo.SelectedIndex - 1);
            }
        };
        Controls.Add(poseCombo);

        angleBox = new TextBox[3];
        string[] ax = { "X 角度", "Y 角度", "Z 角度" };
        for (int i = 0; i < 3; i++)
        {
            Label lab = new Label
            {
                Text = ax[i], Left = x0, Top = 145 + i * 40, Width = 70,
                ForeColor = Color.Gainsboro, BackColor = Color.Transparent
            };
            Controls.Add(lab);
            angleBox[i] = new TextBox
            {
                Left = x0 + 75, Top = 142 + i * 40, Width = 125,
                Text = "0"
            };
            Controls.Add(angleBox[i]);
        }

        applyBtn = new Button
        {
            Text = "应用角度到选中对象",
            Left = x0, Top = 280, Width = 200, Height = 30,
            BackColor = Color.FromArgb(58, 58, 74),
            ForeColor = Color.White,
            FlatStyle = FlatStyle.Flat
        };
        applyBtn.Click += delegate { ApplyAngles(); };
        Controls.Add(applyBtn);

        Button resetBtn = new Button
        {
            Text = "全部复位",
            Left = x0, Top = 320, Width = 200, Height = 30,
            BackColor = Color.FromArgb(58, 58, 74),
            ForeColor = Color.White,
            FlatStyle = FlatStyle.Flat
        };
        resetBtn.Click += delegate { ResetAll(); };
        Controls.Add(resetBtn);

        Button exportBtn = new Button
        {
            Text = "导出状态 CSV",
            Left = x0, Top = 360, Width = 200, Height = 30,
            BackColor = Color.FromArgb(58, 58, 74),
            ForeColor = Color.White,
            FlatStyle = FlatStyle.Flat
        };
        exportBtn.Click += delegate { ExportCsv(); };
        Controls.Add(exportBtn);

        Label tip = new Label
        {
            Text = "拖圆环：绕世界轴转整手\n拖指节：转该关节\n右键拖：转视角",
            Left = x0, Top = 405, Width = 200, Height = 80,
            ForeColor = Color.DimGray, BackColor = Color.Transparent
        };
        Controls.Add(tip);
    }

    static float ParseOrZero(string s)
    {
        float v;
        return float.TryParse(s, out v) ? v : 0f;
    }

    void ApplyAngles()
    {
        float x = ParseOrZero(angleBox[0].Text);
        float y = ParseOrZero(angleBox[1].Text);
        float z = ParseOrZero(angleBox[2].Text);
        const float d2r = (float)Math.PI / 180f;
        SN.Quaternion q = SN.Quaternion.CreateFromAxisAngle(SN.Vector3.UnitY, y * d2r) *
                          SN.Quaternion.CreateFromAxisAngle(SN.Vector3.UnitX, x * d2r) *
                          SN.Quaternion.CreateFromAxisAngle(SN.Vector3.UnitZ, z * d2r);
        int idx = targetCombo.SelectedIndex;
        if (idx <= 0) handRot = SN.Quaternion.Normalize(q);
        else bones[idx].Rot = SN.Quaternion.Normalize(q);
        Invalidate();
    }

    static SN.Quaternion EulerTool(float x, float y, float z)
    {
        const float d2r = (float)Math.PI / 180f;
        SN.Quaternion qy = SN.Quaternion.CreateFromAxisAngle(SN.Vector3.UnitY, y * d2r);
        SN.Quaternion qx = SN.Quaternion.CreateFromAxisAngle(SN.Vector3.UnitX, x * d2r);
        SN.Quaternion qz = SN.Quaternion.CreateFromAxisAngle(SN.Vector3.UnitZ, z * d2r);
        return SN.Quaternion.Multiply(SN.Quaternion.Multiply(qy, qx), qz);
    }

    // 动作预设：每行 16 个关节 × (x, y, z) 欧拉角（tool 顺序 Y-X-Z）
    // y 正 = 向伸直/手背，y 负 = 向掌心弯曲
    static readonly float[][,] PRESETS = new float[][,]
    {
        // 0 张开（标定姿势）
        new float[,] {
            {0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
            {0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0} },
        // 1 握拳（拇指按解剖学：MCP 屈 21°、IP 屈 24°、内收 20°）
        new float[,] {
            {0,0,0},{21f,0f,-20f},{24f,0f,0f},{20f,0f,0f},{0,-80,0},{0,-90,0},{0,-60,0},{0,-80,0},
            {0,-90,0},{0,-60,0},{0,-80,0},{0,-90,0},{0,-60,0},{0,-80,0},{0,-90,0},{0,-60,0} },
        // 2 点赞（拇指伸，四指握拳）
        new float[,] {
            {0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,-80,0},{0,-90,0},{0,-60,0},{0,-80,0},
            {0,-90,0},{0,-60,0},{0,-80,0},{0,-90,0},{0,-60,0},{0,-80,0},{0,-90,0},{0,-60,0} },
        // 3 挥手（手指伸直，hand 绕 z 摆）
        new float[,] {
            {0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
            {0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0} },
        // 4 OK（拇指+食指成环）
        new float[,] {
            {0,0,0},{0,-40,0},{0,-40,0},{0,0,0},{0,-50,0},{0,-50,0},{0,0,0},{0,0,0},
            {0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0} },
        // 5 比耶（食指+中指伸）
        new float[,] {
            {0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
            {0,0,0},{0,0,0},{0,-80,0},{0,-90,0},{0,-60,0},{0,-80,0},{0,-90,0},{0,-60,0} },
        // 6 竖中指
        new float[,] {
            {0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,-80,0},{0,-90,0},{0,-60,0},{0,0,0},
            {0,0,0},{0,0,0},{0,-80,0},{0,-90,0},{0,-60,0},{0,-80,0},{0,-90,0},{0,-60,0} },
    };

    static SN.Quaternion EulerToolZYX(float x, float y, float z)
    {
        const float d2r = (float)Math.PI / 180f;
        SN.Quaternion rx = SN.Quaternion.CreateFromAxisAngle(SN.Vector3.UnitX, x * d2r);
        SN.Quaternion ry = SN.Quaternion.CreateFromAxisAngle(SN.Vector3.UnitY, y * d2r);
        SN.Quaternion rz = SN.Quaternion.CreateFromAxisAngle(SN.Vector3.UnitZ, z * d2r);
        // R = Rz * Ry * Rx（与 QuatToEuler 的分解顺序一致）
        return SN.Quaternion.Multiply(SN.Quaternion.Multiply(rz, ry), rx);
    }

    void ApplyPreset(int idx)
    {
        if (idx < 0 || idx >= PRESETS.Length)
        {
            return;
        }
        float[,] d = PRESETS[idx];
        handRot = SN.Quaternion.Identity;
        for (int i = 1; i < 16; i++)
        {
            bones[i].Rot = EulerToolZYX(d[i, 0], d[i, 1], d[i, 2]);
        }
        // 挥手：手腕左右摆 45°
        if (idx == 3)
        {
            handRot = EulerTool(0, 0, 45);
        }
        Invalidate();
    }

    void ResetAll()
    {
        handRot = SN.Quaternion.Identity;
        for (int i = 1; i < 16; i++) bones[i].Rot = SN.Quaternion.Identity;
        for (int i = 0; i < 3; i++) angleBox[i].Text = "0";
        Invalidate();
    }

    static SN.Vector3 QuatToEuler(SN.Quaternion q)
    {
        float sinr = 2f * (q.W * q.X + q.Y * q.Z);
        float cosr = 1f - 2f * (q.X * q.X + q.Y * q.Y);
        float rx = (float)Math.Atan2(sinr, cosr);
        float sinp = 2f * (q.W * q.Y - q.Z * q.X);
        if (sinp > 1f) sinp = 1f;
        if (sinp < -1f) sinp = -1f;
        float ry = (float)Math.Asin(sinp);
        float siny = 2f * (q.W * q.Z + q.X * q.Y);
        float cosy = 1f - 2f * (q.Y * q.Y + q.Z * q.Z);
        float rz = (float)Math.Atan2(siny, cosy);
        const float r2d = 180f / (float)Math.PI;
        return new SN.Vector3(rx * r2d, ry * r2d, rz * r2d);
    }

    void ExportCsv()
    {
        SN.Quaternion hand = handRot;
        StringBuilder sb = new StringBuilder();
        sb.AppendLine("joint,px,py,pz,rx,ry,rz,lx,ly,lz");
        for (int i = 0; i < 16; i++)
        {
            SN.Vector3 p = WorldPos(i, hand);
            SN.Vector3 e = QuatToEuler(WorldRot(i, hand));
            SN.Vector3 l = QuatToEuler(bones[i].Rot);
            sb.AppendLine(string.Format("{0},{1:F4},{2:F4},{3:F4},{4:F1},{5:F1},{6:F1},{7:F1},{8:F1},{9:F1}",
                names[i], p.X, p.Y, p.Z, e.X, e.Y, e.Z, l.X, l.Y, l.Z));
        }
        string dir = System.IO.Path.GetDirectoryName(Application.ExecutablePath);
        string path = System.IO.Path.Combine(dir, "pose_state.csv");
        System.IO.File.WriteAllText(path, sb.ToString());
        MessageBox.Show("已导出：" + path);
    }

    // ---------- 骨架 ----------

    void InitSkeleton()
    {
        // 手掌：扁平长方体，手背朝 +z（天空），手指朝 -x；y 方向是手宽（拇指 -y，小指 +y）
        bones[0] = new Bone
        {
            Name = "hand", Parent = -1, Offset = SN.Vector3.Zero, Axis = -ELBOW,
            Length = 0.075f, Width = 0.075f,
            BoxCenter = new SN.Vector3(-0.0375f, -0.0035f, 0f),
            BoxHalf = new SN.Vector3(0.0375f, 0.050f, 0.012f)
        };

        // 拇指：从手掌左侧(-y)边缘斜着伸出，相对 -x 外张约 30°
        AddChain(1, 0, new SN.Vector3(-0.012f, -0.045f, 0f), SN.Vector3.Normalize(new SN.Vector3(-0.866f, -0.5f, 0f)),
                 new float[] { 0.034f, 0.03f, 0.024f }, new float[] { 0.022f, 0.019f, 0.016f });
        // 四指：间距 0.023，指根 z 有 22° 横弓（中指最高、小指最低，弧深约 4mm）
        // 张角按放松态：食指 -5°、中指 0°、无名指 +5°、小指 +12°
        AddChain(4, 0, new SN.Vector3(-0.075f, -0.0345f, -0.0020f), SN.Vector3.Normalize(new SN.Vector3(-1f, -0.087f, 0f)),
                 new float[] { 0.042f, 0.032f, 0.026f }, new float[] { 0.021f, 0.018f, 0.015f });
        AddChain(7, 0, new SN.Vector3(-0.075f, -0.0115f, 0f), -ELBOW,
                 new float[] { 0.045f, 0.034f, 0.027f }, new float[] { 0.022f, 0.019f, 0.015f });
        AddChain(10, 0, new SN.Vector3(-0.075f, 0.0115f, -0.0015f), SN.Vector3.Normalize(new SN.Vector3(-1f, 0.087f, 0f)),
                 new float[] { 0.042f, 0.032f, 0.026f }, new float[] { 0.021f, 0.018f, 0.015f });
        AddChain(13, 0, new SN.Vector3(-0.073f, 0.0345f, -0.0040f), SN.Vector3.Normalize(new SN.Vector3(-1f, 0.21f, 0f)),
                 new float[] { 0.034f, 0.026f, 0.021f }, new float[] { 0.019f, 0.016f, 0.013f });
    }

    void AddChain(int start, int parent, SN.Vector3 offset, SN.Vector3 axis, float[] lengths, float[] widths)
    {
        int p = parent;
        for (int i = 0; i < 3; i++)
        {
            int idx = start + i;
            bones[idx] = new Bone
            {
                Name = names[idx],
                Parent = p,
                Offset = (i == 0) ? offset : bones[p].Axis * bones[p].Length,
                Axis = axis,
                Length = lengths[i],
                Width = widths[i]
            };
            p = idx;
        }
    }

    // ---------- 变换 ----------

    SN.Quaternion WorldRot(int i, SN.Quaternion hand)
    {
        int p = bones[i].Parent;
        return p < 0 ? hand : SN.Quaternion.Normalize(WorldRot(p, hand) * bones[i].Rot);
    }

    SN.Vector3 WorldPos(int i, SN.Quaternion hand)
    {
        int p = bones[i].Parent;
        if (p < 0) return SN.Vector3.Zero;
        return WorldPos(p, hand) + SN.Vector3.Transform(bones[i].Offset, WorldRot(p, hand));
    }

    // ---------- 相机 ----------

    SN.Vector3 Eye()
    {
        SN.Vector3 dir = new SN.Vector3(
            (float)(Math.Cos(pitch) * Math.Cos(yaw)),
            (float)(Math.Cos(pitch) * Math.Sin(yaw)),
            (float)Math.Sin(pitch));
        return dir * distance;
    }

    void CameraBasis(out SN.Vector3 eye, out SN.Vector3 right, out SN.Vector3 up, out SN.Vector3 forward)
    {
        eye = Eye();
        forward = SN.Vector3.Normalize(-eye);
        SN.Vector3 cr = SN.Vector3.Cross(forward, UP);
        if (cr.Length() < 0.01f) cr = SN.Vector3.Cross(forward, SN.Vector3.UnitX);
        right = SN.Vector3.Normalize(cr);
        up = SN.Vector3.Cross(right, forward);
    }

    PointF Project(SN.Vector3 p, out float depth)
    {
        SN.Vector3 eye, right, up, forward;
        CameraBasis(out eye, out right, out up, out forward);
        SN.Vector3 rel = p - eye;
        float scale = 1300f;
        float viewW = 780f;
        float viewH = 760f;
        float u = SN.Vector3.Dot(rel, right) * scale + viewW * 0.5f;
        float v = -SN.Vector3.Dot(rel, up) * scale + viewH * 0.5f;
        depth = SN.Vector3.Dot(rel, forward);
        return new PointF(u, v);
    }

    // ---------- 渲染 ----------

    class Face
    {
        public PointF[] Pts;
        public float Depth;
        public Color Fill;
        public Color Edge;
        public float EdgeWidth;
    }

    static Color Shade(Color c, float s)
    {
        return Color.FromArgb(c.A,
            Math.Min(255, (int)(c.R * s)),
            Math.Min(255, (int)(c.G * s)),
            Math.Min(255, (int)(c.B * s)));
    }

    static Color FingerColor(int idx)
    {
        int f = idx <= 0 ? 0 : (idx - 1) / 3 + 1;
        return fingerColors[Math.Min(f, fingerColors.Length - 1)];
    }

    static SN.Quaternion LookRot(SN.Vector3 dir, SN.Vector3 upHint)
    {
        SN.Vector3 z = SN.Vector3.Normalize(dir);
        SN.Vector3 x = SN.Vector3.Cross(upHint, z);
        if (x.Length() < 0.01f)
        {
            x = SN.Vector3.Cross(SN.Vector3.UnitX, z);
        }
        x = SN.Vector3.Normalize(x);
        SN.Vector3 y = SN.Vector3.Cross(z, x);
        SN.Matrix4x4 m = new SN.Matrix4x4(
            x.X, x.Y, x.Z, 0f,
            y.X, y.Y, y.Z, 0f,
            z.X, z.Y, z.Z, 0f,
            0f, 0f, 0f, 1f);
        return SN.Quaternion.CreateFromRotationMatrix(m);
    }

    void AddCylinder(List<Face> faces, SN.Vector3 a, SN.Vector3 b, float r, int idx, SN.Vector3 light, bool selected)
    {
        SN.Vector3 dir = SN.Vector3.Normalize(b - a);
        SN.Vector3 helper = Math.Abs(dir.Z) < 0.9f ? SN.Vector3.UnitZ : SN.Vector3.UnitY;
        SN.Vector3 u = SN.Vector3.Normalize(SN.Vector3.Cross(dir, helper));
        SN.Vector3 v = SN.Vector3.Cross(dir, u);
        const int N = 12;
        Color baseCol = FingerColor(idx);
        if (selected) baseCol = Color.FromArgb(255, 255, 255, 255);

        for (int i = 0; i < N; i++)
        {
            float a0 = (float)(i * Math.PI * 2 / N);
            float a1 = (float)((i + 1) * Math.PI * 2 / N);
            float am = (a0 + a1) * 0.5f;
            SN.Vector3 n = u * (float)Math.Cos(am) + v * (float)Math.Sin(am);
            SN.Vector3 p0 = a + (u * (float)Math.Cos(a0) + v * (float)Math.Sin(a0)) * r;
            SN.Vector3 p1 = a + (u * (float)Math.Cos(a1) + v * (float)Math.Sin(a1)) * r;
            SN.Vector3 p2 = b + (u * (float)Math.Cos(a1) + v * (float)Math.Sin(a1)) * r;
            SN.Vector3 p3 = b + (u * (float)Math.Cos(a0) + v * (float)Math.Sin(a0)) * r;
            float shade = 0.42f + 0.62f * Math.Max(0f, SN.Vector3.Dot(n, light));
            float d0, d1, d2, d3;
            PointF q0 = Project(p0, out d0);
            PointF q1 = Project(p1, out d1);
            PointF q2 = Project(p2, out d2);
            PointF q3 = Project(p3, out d3);
            faces.Add(new Face
            {
                Pts = new[] { q0, q1, q2, q3 },
                Depth = (d0 + d1 + d2 + d3) * 0.25f,
                Fill = Shade(baseCol, shade),
                Edge = Color.FromArgb(22, 20, 20, 30),
                EdgeWidth = 0.6f
            });
        }
    }

    void AddBox(List<Face> faces, SN.Vector3 center, SN.Vector3 half, SN.Quaternion rot, int idx, SN.Vector3 light, bool selected)
    {
        int[][] quads = {
            new[]{0,1,3,2}, new[]{4,6,7,5}, new[]{0,2,6,4},
            new[]{1,5,7,3}, new[]{0,4,5,1}, new[]{2,3,7,6}
        };
        SN.Vector3[] normals = {
            new SN.Vector3(0,0,-1), new SN.Vector3(0,0,1), new SN.Vector3(-1,0,0),
            new SN.Vector3(1,0,0), new SN.Vector3(0,-1,0), new SN.Vector3(0,1,0)
        };
        Color baseCol = FingerColor(idx);
        if (selected) baseCol = Color.White;
        for (int f = 0; f < 6; f++)
        {
            SN.Vector3 n = SN.Vector3.Transform(normals[f], rot);
            float shade = 0.42f + 0.62f * Math.Max(0f, SN.Vector3.Dot(n, light));
            PointF[] pts = new PointF[4];
            float dsum = 0;
            for (int k = 0; k < 4; k++)
            {
                int c = quads[f][k];
                SN.Vector3 v = new SN.Vector3(
                    ((c & 1) == 0 ? -half.X : half.X),
                    ((c & 2) == 0 ? -half.Y : half.Y),
                    ((c & 4) == 0 ? -half.Z : half.Z));
                float d;
                pts[k] = Project(center + SN.Vector3.Transform(v, rot), out d);
                dsum += d;
            }
            faces.Add(new Face
            {
                Pts = pts,
                Depth = dsum * 0.25f,
                Fill = Shade(baseCol, shade),
                Edge = Color.FromArgb(26, 20, 20, 30),
                EdgeWidth = 0.7f
            });
        }
    }

    void AddSphere(List<Face> faces, SN.Vector3 c, float r, Color col, float depthBias)
    {
        float d;
        PointF p = Project(c, out d);
        float rr = Math.Max(3.0f, r * 1300f);
        faces.Add(new Face
        {
            Pts = CirclePts(p, rr),
            Depth = d + depthBias,
            Fill = Shade(col, 0.85f),
            Edge = Color.FromArgb(90, 20, 20, 30),
            EdgeWidth = 1f
        });
        // 高光
        faces.Add(new Face
        {
            Pts = CirclePts(new PointF(p.X - rr * 0.28f, p.Y - rr * 0.28f), rr * 0.42f),
            Depth = d + depthBias - 0.002f,
            Fill = Color.FromArgb(110, 255, 255, 255),
            Edge = Color.Transparent,
            EdgeWidth = 0f
        });
    }

    static PointF[] CirclePts(PointF c, float r)
    {
        PointF[] pts = new PointF[20];
        for (int i = 0; i < 20; i++)
        {
            double a = i * Math.PI * 2 / 20;
            pts[i] = new PointF(c.X + (float)(Math.Cos(a) * r), c.Y + (float)(Math.Sin(a) * r));
        }
        return pts;
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        Graphics g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        g.Clear(BackColor);

        // 固定的世界坐标轴：原点在手腕，x+ 肘、y+ 右、z+ 天（与固件一致）
        DrawWorldAxes(g);

        SN.Quaternion hand = handRot;
        List<Face> faces = new List<Face>();
        SN.Vector3 light = SN.Vector3.Normalize(new SN.Vector3(-0.45f, -0.55f, 1f));

        for (int i = 0; i < 16; i++)
        {
            Bone b = bones[i];
            SN.Vector3 pos = WorldPos(i, hand);
            SN.Quaternion rot = WorldRot(i, hand);
            SN.Vector3 axis = SN.Vector3.Transform(b.Axis, rot);
            SN.Vector3 start = pos;
            SN.Vector3 end = pos + axis * b.Length;

            if (b.BoxHalf != SN.Vector3.Zero)
            {
                SN.Vector3 center = pos + SN.Vector3.Transform(b.BoxCenter, rot);
                AddBox(faces, center, b.BoxHalf, rot, i, light, dragBone == i);
            }
            else
            {
                // 指节用长方体：有清晰的面和棱，能看出绕指骨轴的扭转
                SN.Vector3 segDir = end - start;
                AddBox(faces, (start + end) * 0.5f,
                       new SN.Vector3(b.Width * 0.5f, b.Width * 0.25f, segDir.Length() * 0.5f),
                       LookRot(segDir, SN.Vector3.UnitZ), i, light, dragBone == i);
            }
            AddSphere(faces, start, b.Width * 0.46f, FingerColor(i), 0f);
            if (i == 3 || i == 6 || i == 9 || i == 12 || i == 15)
            {
                AddSphere(faces, end, b.Width * 0.4f, FingerColor(i), -0.001f);
            }
        }

        faces.Sort(delegate (Face x, Face y) { return y.Depth.CompareTo(x.Depth); });
        foreach (Face f in faces)
        {
            using (SolidBrush br = new SolidBrush(f.Fill))
            {
                g.FillPolygon(br, f.Pts);
                if (f.EdgeWidth > 0f)
                {
                    using (Pen pen = new Pen(f.Edge, f.EdgeWidth))
                    {
                        g.DrawPolygon(pen, f.Pts);
                    }
                }
            }
        }

        DrawRings(g);
        DrawHud(g);
        DrawJointInfo(g);
        DrawStateTable(g);
    }

    // ---------- 世界坐标轴（固定原点，标定基准） ----------

    void DrawWorldAxes(Graphics g)
    {
        SN.Vector3 o = SN.Vector3.Zero;
        SN.Vector3[] dirs = { SN.Vector3.UnitX, SN.Vector3.UnitY, SN.Vector3.UnitZ };
        Color[] cols = { Color.FromArgb(230, 80, 80), Color.FromArgb(80, 220, 100), Color.FromArgb(90, 150, 255) };
        string[] labels = { "x+ 肘", "y+ 右", "z+ 天" };
        float dummy;
        PointF p0 = Project(o, out dummy);
        for (int i = 0; i < 3; i++)
        {
            float d1;
            PointF p1 = Project(dirs[i] * 0.16f, out d1);
            using (Pen pen = new Pen(cols[i], 2.5f))
            {
                g.DrawLine(pen, p0, p1);
            }
            using (SolidBrush br = new SolidBrush(cols[i]))
            {
                g.FillEllipse(br, p1.X - 4, p1.Y - 4, 8, 8);
                g.DrawString(labels[i], small, br, p1.X + 6f, p1.Y - 6f);
            }
        }
        using (SolidBrush br = new SolidBrush(Color.FromArgb(235, 255, 255, 255)))
        {
            g.FillEllipse(br, p0.X - 4, p0.Y - 4, 8, 8);
        }
    }

    // ---------- HUD ----------

    // ---------- 三轴圆环 ----------

    const float RING_R = 58f;

    PointF RingCenter()
    {
        return new PointF(780f - 95f, 760f - 95f);
    }

    static float RingAngle(int axis, float t)
    {
        return t;
    }

    // 每个环用椭圆近似：x 环（绕 X 轴）水平压扁，y 环（绕 Y）垂直压扁，z 环是正圆
    void RingRadii(int axis, float r, out float rx, out float ry)
    {
        if (axis == 0) { rx = r * 0.36f; ry = r; }
        else if (axis == 1) { rx = r; ry = r * 0.36f; }
        else { rx = r; ry = r; }
    }

    void DrawRings(Graphics g)
    {
        PointF c = RingCenter();
        Color[] cols = { Color.FromArgb(240, 80, 80), Color.FromArgb(80, 235, 100), Color.FromArgb(90, 150, 255) };
        for (int a = 0; a < 3; a++)
        {
            float rx, ry;
            RingRadii(a, RING_R, out rx, out ry);
            using (Pen pen = new Pen(cols[a], 2.4f))
            {
                g.DrawEllipse(pen, c.X - rx, c.Y - ry, rx * 2, ry * 2);
            }
        }
        using (SolidBrush br = new SolidBrush(Color.FromArgb(230, 255, 255, 255)))
        {
            g.FillEllipse(br, c.X - 3, c.Y - 3, 6, 6);
        }
        using (SolidBrush br = new SolidBrush(Color.FromArgb(160, 200, 200, 215)))
        {
            g.DrawString("x 环 / y 环 / z 环", small, br, c.X - 52, c.Y + RING_R + 6);
        }
    }

    int HitRing(Point p)
    {
        PointF c = RingCenter();
        int best = -1;
        float bestD = 12f;
        for (int a = 0; a < 3; a++)
        {
            float rx, ry;
            RingRadii(a, RING_R, out rx, out ry);
            for (int i = 0; i < 72; i++)
            {
                float t = i * (float)Math.PI * 2 / 72;
                float x = c.X + rx * (float)Math.Cos(t);
                float y = c.Y + ry * (float)Math.Sin(t);
                float dx = x - p.X;
                float dy = y - p.Y;
                float d = (float)Math.Sqrt(dx * dx + dy * dy);
                if (d < bestD)
                {
                    bestD = d;
                    best = a;
                }
            }
        }
        return best;
    }

    void DrawHud(Graphics g)
    {
        using (SolidBrush br = new SolidBrush(Color.FromArgb(220, 220, 235)))
        {
            g.DrawString(
                string.Format("UDP {0}:{1}   {2} Hz   已发 {3} 包", host, port, (int)rate, sent),
                font, br, new PointF(16, 12));
            g.DrawString(
                "拖指节=转该关节   拖右下角圆环=绕世界轴转整手   右键拖=转视角   R=复位   Esc=退出",
                small, br, new PointF(16, 40));
            g.DrawString("坐标：x+ 肘部 / y+ 身体右 / z+ 天空（与固件机体系一致）", small, br, new PointF(16, 60));
        }
    }

    void DrawJointInfo(Graphics g)
    {
        int y = 90;
        using (SolidBrush br = new SolidBrush(Color.FromArgb(180, 200, 200, 215)))
        {
            g.DrawString("关节（相对父关节的旋转角）", small, br, new PointF(16, y));
            y += 20;
            for (int i = 1; i < 16; i++)
            {
                SN.Quaternion q = bones[i].Rot;
                float ang = (float)(2.0 * Math.Acos(Math.Min(1.0, Math.Abs(q.W))) * 180.0 / Math.PI);
                Color c = FingerColor(i);
                using (SolidBrush cb = new SolidBrush(c))
                    g.DrawString(names[i], small, cb, new PointF(16, y));
                g.DrawString(string.Format("{0,6:F1}°", ang), small, br, new PointF(110, y));
                y += 16;
            }
        }
    }

    // ---------- 状态表（每个关节的世界位置/欧拉角） ----------

    void DrawStateTable(Graphics g)
    {
        float y0 = 760f;
        using (Pen pen = new Pen(Color.FromArgb(70, 70, 85), 1f))
        {
            g.DrawLine(pen, 0, y0, ClientSize.Width, y0);
        }

        SN.Quaternion hand = handRot;
        using (SolidBrush head = new SolidBrush(Color.FromArgb(160, 160, 180)))
        {
            g.DrawString("关节", small, head, new PointF(8, y0 + 3));
            g.DrawString("位置（x,y,z）", small, head, new PointF(86, y0 + 3));
            g.DrawString("欧拉角（x,y,z °）", small, head, new PointF(248, y0 + 3));
            g.DrawString("关节", small, head, new PointF(510, y0 + 3));
            g.DrawString("位置（x,y,z）", small, head, new PointF(588, y0 + 3));
            g.DrawString("欧拉角（x,y,z °）", small, head, new PointF(750, y0 + 3));
        }

        for (int i = 0; i < 16; i++)
        {
            int col = i / 8;
            int row = i % 8;
            float x = (col == 0) ? 8f : 510f;
            float y = y0 + 20f + row * 16f;
            SN.Vector3 p = WorldPos(i, hand);
            SN.Vector3 e = QuatToEuler(WorldRot(i, hand));
            Color c = FingerColor(i);
            using (SolidBrush nb = new SolidBrush(c))
            {
                g.DrawString(names[i], small, nb, new PointF(x, y));
            }
            using (SolidBrush vb = new SolidBrush(Color.FromArgb(200, 200, 200, 215)))
            {
                g.DrawString(string.Format("{0,6:F3} {1,6:F3} {2,6:F3}", p.X, p.Y, p.Z), small, vb, new PointF(x + 78, y));
                g.DrawString(string.Format("{0,5:F0} {1,5:F0} {2,5:F0}", e.X, e.Y, e.Z), small, vb, new PointF(x + 240, y));
            }
        }
    }

    // ---------- 交互 ----------

    protected override void OnMouseDown(MouseEventArgs e)
    {
        base.OnMouseDown(e);
        dragLast = e.Location;

        if (e.Button == MouseButtons.Right)
        {
            return;
        }

        int hit = HitRing(e.Location);
        if (hit >= 0)
        {
            ringAxis = hit;
            PointF rc = RingCenter();
            ringLastAngle = (float)Math.Atan2(e.Y - rc.Y, e.X - rc.X);
            return;
        }

        float best = 24f;
        int bestIdx = -1;
        SN.Quaternion hand = handRot;
        for (int i = 0; i < 16; i++)
        {
            float d;
            PointF p = Project(WorldPos(i, hand), out d);
            float dx = p.X - e.X;
            float dy = p.Y - e.Y;
            float dist = (float)Math.Sqrt(dx * dx + dy * dy);
            if (dist < best)
            {
                best = dist;
                bestIdx = i;
            }
        }
        dragBone = bestIdx;
        Invalidate();
    }

    protected override void OnMouseMove(MouseEventArgs e)
    {
        base.OnMouseMove(e);
        float dx = e.X - dragLast.X;
        float dy = e.Y - dragLast.Y;
        dragLast = e.Location;

        if (e.Button == MouseButtons.Right)
        {
            yaw += dx * 0.01f;
            pitch += dy * 0.01f;
            Invalidate();
            return;
        }

        if (e.Button != MouseButtons.Left) return;

        if (ringAxis >= 0)
        {
            PointF rc = RingCenter();
            float now = (float)Math.Atan2(e.Y - rc.Y, e.X - rc.X);
            float delta = now - ringLastAngle;
            ringLastAngle = now;
            SN.Vector3 axis = ringAxis == 0 ? SN.Vector3.UnitX : (ringAxis == 1 ? SN.Vector3.UnitY : SN.Vector3.UnitZ);
            handRot = SN.Quaternion.Normalize(SN.Quaternion.CreateFromAxisAngle(axis, delta) * handRot);
            Invalidate();
            return;
        }

        SN.Vector3 eye, right, up, forward;
        CameraBasis(out eye, out right, out up, out forward);
        SN.Quaternion r = SN.Quaternion.CreateFromAxisAngle(up, dx * 0.008f) *
                          SN.Quaternion.CreateFromAxisAngle(right, dy * 0.008f);

        if (dragBone > 0)
        {
            bones[dragBone].Rot = SN.Quaternion.Normalize(bones[dragBone].Rot * r);
        }
        Invalidate();
    }

    protected override void OnMouseUp(MouseEventArgs e)
    {
        base.OnMouseUp(e);
        dragBone = -1;
        ringAxis = -1;
        Invalidate();
    }

    protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
    {
        if (keyData == Keys.R)
        {
            ResetAll();
            return true;
        }
        if (keyData == Keys.Escape)
        {
            Close();
            return true;
        }
        return base.ProcessCmdKey(ref msg, keyData);
    }

    // ---------- 发送 ----------

    static void WriteQuat(List<float> buf, SN.Quaternion q)
    {
        q = SN.Quaternion.Normalize(q);
        buf.Add(q.W);
        buf.Add(q.X);
        buf.Add(q.Y);
        buf.Add(q.Z);
    }

    byte[] BuildPacket(long seq)
    {
        List<float> qs = new List<float>(64);
        WriteQuat(qs, handRot);
        for (int i = 1; i < 16; i++) WriteQuat(qs, bones[i].Rot);

        System.IO.MemoryStream ms = new System.IO.MemoryStream(272);
        System.IO.BinaryWriter bw = new System.IO.BinaryWriter(ms);
        bw.Write((byte)'E'); bw.Write((byte)'I'); bw.Write((byte)'G'); bw.Write((byte)'V');
        bw.Write(VERSION);
        bw.Write((byte)hand);
        bw.Write((byte)0);
        bw.Write((uint)seq);
        bw.Write((uint)(DateTime.UtcNow.Ticks / TimeSpan.TicksPerMillisecond));
        for (int i = 0; i < 64; i++) bw.Write(qs[i]);
        bw.Flush();
        byte[] data = ms.ToArray();
        bw.Close();
        ms.Close();
        return data;
    }

    void SendLoop()
    {
        UdpClient udp;
        try { udp = new UdpClient(); } catch { return; }
        IPEndPoint ep = new IPEndPoint(IPAddress.Parse(host), port);
        long seq = 0;
        double period = 1.0 / SEND_HZ;
        System.Diagnostics.Stopwatch sw = System.Diagnostics.Stopwatch.StartNew();
        while (running)
        {
            byte[] pkt = BuildPacket(seq);
            try
            {
                udp.Send(pkt, pkt.Length, ep);
                Interlocked.Increment(ref sent);
            }
            catch { }
            seq++;
            double target = seq * period;
            double elapsed = sw.Elapsed.TotalSeconds;
            if (target > elapsed) Thread.Sleep((int)Math.Max(0, (target - elapsed) * 1000.0));
        }
        try { udp.Close(); } catch { }
    }

    protected override void OnFormClosing(FormClosingEventArgs e)
    {
        running = false;
        try { if (sender != null && sender.IsAlive) sender.Join(200); } catch { }
        timer.Stop();
        base.OnFormClosing(e);
    }
}
