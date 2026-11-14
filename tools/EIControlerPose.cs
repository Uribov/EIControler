// EIControlerPose —— 手部姿态控制器
//
// 在 2D 骨骼模型上拖指尖，按 EIGV 协议把 16 关节四元数发给 VaM 插件
// （默认 127.0.0.1:9001，200Hz）。用来手动验证 VaM 端的骨骼映射和朝向补偿。
//
// 编译（Windows 自带 .NET Framework 编译器，无需额外依赖）：
//   csc /target:winexe /optimize+ /r:System.Windows.Forms.dll /r:System.Drawing.dll
//       /out:EIControlerPose.exe EIControlerPose.cs
//
// 协议：4 字节 "EIGV" + <HBBII>(ver, hand, 0, seq, ts_ms) + 16 × 4 float，共 272 字节。

using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Windows.Forms;

static class Program
{
    [STAThread]
    static void Main(string[] args)
    {
        string host = "127.0.0.1";
        int port = 9001;
        int hand = 1;
        string poseArg = null;

        for (int i = 0; i < args.Length; i++)
        {
            if (args[i] == "--host" && i + 1 < args.Length) host = args[++i];
            else if (args[i] == "--udp-port" && i + 1 < args.Length) int.TryParse(args[++i], out port);
            else if (args[i] == "--hand" && i + 1 < args.Length) int.TryParse(args[++i], out hand);
            else if (args[i] == "--pose" && i + 1 < args.Length) poseArg = args[++i];
        }

        float[] initBend = null;
        if (!string.IsNullOrEmpty(poseArg))
        {
            string[] parts = poseArg.Split(',');
            initBend = new float[5];
            for (int i = 0; i < 5 && i < parts.Length; i++)
            {
                float v;
                if (float.TryParse(parts[i], out v)) initBend[i] = v;
            }
        }

        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        Application.Run(new PoseForm(host, port, hand, initBend));
    }
}

class PoseForm : Form
{
    const string MAGIC = "EIGV";
    const ushort VERSION = 1;
    const double SEND_HZ = 200.0;

    static readonly string[] FINGER_NAMES = { "拇指", "食指", "中指", "无名指", "小指" };
    static readonly int[] BASE_JOINT = { 1, 4, 7, 10, 13 };   // thumb,index,middle,ring,pinky
    static readonly float[] SEG_BEND = { 45f, 75f, 55f };     // 每节最大弯曲（度）

    static readonly PointF[] BASE_POS = {
        new PointF(285f, 490f),
        new PointF(330f, 425f),
        new PointF(395f, 415f),
        new PointF(460f, 425f),
        new PointF(525f, 440f),
    };
    static readonly float[] BASE_DEG = { -105f, -90f, -90f, -90f, -90f };
    static readonly float[][] LENGTHS = {
        new float[] { 62f, 50f, 40f },
        new float[] { 70f, 55f, 45f },
        new float[] { 76f, 58f, 46f },
        new float[] { 70f, 55f, 45f },
        new float[] { 60f, 48f, 40f },
    };

    static readonly Color BG = Color.FromArgb(30, 30, 40);
    static readonly Color PALM_FILL = Color.FromArgb(58, 58, 74);
    static readonly Color PALM_LINE = Color.FromArgb(90, 90, 112);
    static readonly Color BONE = Color.FromArgb(232, 232, 240);
    static readonly Color JOINT = Color.FromArgb(142, 197, 255);
    static readonly Color TIP = Color.FromArgb(255, 211, 77);
    static readonly Color TIP_DRAG = Color.FromArgb(255, 122, 69);
    static readonly Color TEXT = Color.FromArgb(200, 200, 216);
    static readonly Color DIM = Color.FromArgb(138, 138, 160);

    readonly float[] bend = new float[5];
    readonly object bendLock = new object();
    int drag = -1;
    long sent;
    float rate;
    long lastCount;
    DateTime lastRateTime = DateTime.Now;

    readonly string host;
    readonly int port;
    readonly int hand;
    volatile bool running = true;
    Thread sender;

    readonly Font font;
    readonly Font small;
    readonly System.Windows.Forms.Timer timer;

    public PoseForm(string host, int port, int hand, float[] initBend)
    {
        this.host = host;
        this.port = port;
        this.hand = hand;

        if (initBend != null)
        {
            for (int i = 0; i < 5; i++) bend[i] = initBend[i];
        }

        Text = "EIGlove 手部姿态控制器";
        ClientSize = new Size(760, 620);
        FormBorderStyle = FormBorderStyle.FixedSingle;
        MaximizeBox = false;
        StartPosition = FormStartPosition.CenterScreen;
        BackColor = BG;
        DoubleBuffered = true;
        KeyPreview = true;

        font = new Font("Microsoft YaHei UI", 11f);
        small = new Font("Microsoft YaHei UI", 9f);

        timer = new System.Windows.Forms.Timer();
        timer.Interval = 33;
        timer.Tick += delegate
        {
            float nowRate = (float)((sent - lastCount) / Math.Max((DateTime.Now - lastRateTime).TotalSeconds, 0.001));
            if ((DateTime.Now - lastRateTime).TotalSeconds >= 0.5)
            {
                rate = nowRate;
                lastCount = sent;
                lastRateTime = DateTime.Now;
            }
            Invalidate();
        };
        timer.Start();

        sender = new Thread(SendLoop);
        sender.IsBackground = true;
        sender.Start();
    }

    // ---------- 几何 ----------

    PointF[] FK(int finger, float c)
    {
        PointF[] pts = new PointF[4];
        pts[0] = BASE_POS[finger];
        float acc = 0f;
        float baseDeg = BASE_DEG[finger];
        for (int seg = 0; seg < 3; seg++)
        {
            acc += SEG_BEND[seg] * c;
            double ang = (baseDeg + acc) * Math.PI / 180.0;
            float len = LENGTHS[finger][seg];
            pts[seg + 1] = new PointF(
                pts[seg].X + (float)(len * Math.Cos(ang)),
                pts[seg].Y + (float)(len * Math.Sin(ang)));
        }
        return pts;
    }

    float BendForPoint(int finger, float mx, float my)
    {
        float bestC = 0f;
        float bestD = float.MaxValue;
        for (int k = 0; k <= 100; k++)
        {
            float c = k / 100f;
            PointF tip = FK(finger, c)[3];
            float dx = tip.X - mx;
            float dy = tip.Y - my;
            float d = dx * dx + dy * dy;
            if (d < bestD)
            {
                bestD = d;
                bestC = c;
            }
        }
        return bestC;
    }

    // ---------- 交互 ----------

    protected override void OnMouseDown(MouseEventArgs e)
    {
        base.OnMouseDown(e);
        float bestD = 40f * 40f;
        int best = -1;
        for (int i = 0; i < 5; i++)
        {
            PointF tip = FK(i, bend[i])[3];
            float dx = tip.X - e.X;
            float dy = tip.Y - e.Y;
            float d = dx * dx + dy * dy;
            if (d < bestD)
            {
                bestD = d;
                best = i;
            }
        }
        if (best >= 0)
        {
            drag = best;
            Invalidate();
        }
    }

    protected override void OnMouseMove(MouseEventArgs e)
    {
        base.OnMouseMove(e);
        if (drag < 0) return;
        lock (bendLock)
        {
            bend[drag] = BendForPoint(drag, e.X, e.Y);
        }
        Invalidate();
    }

    protected override void OnMouseUp(MouseEventArgs e)
    {
        base.OnMouseUp(e);
        drag = -1;
        Invalidate();
    }

    protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
    {
        if (keyData == Keys.R || keyData == (Keys.Shift | Keys.R))
        {
            lock (bendLock)
            {
                for (int i = 0; i < 5; i++) bend[i] = 0f;
            }
            Invalidate();
            return true;
        }
        if (keyData == Keys.Escape)
        {
            Close();
            return true;
        }
        return base.ProcessCmdKey(ref msg, keyData);
    }

    // ---------- 绘制 ----------

    protected override void OnPaint(PaintEventArgs e)
    {
        Graphics g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        g.Clear(BG);

        using (SolidBrush palmBrush = new SolidBrush(PALM_FILL))
        using (Pen palmPen = new Pen(PALM_LINE, 3f))
        {
            RectangleF rect = new RectangleF(250f, 425f, 310f, 150f);
            g.FillRectangle(palmBrush, rect);
            g.DrawRectangle(palmPen, rect.X, rect.Y, rect.Width, rect.Height);
        }

        using (SolidBrush dimBrush = new SolidBrush(DIM))
        using (StringFormat sf = new StringFormat())
        {
            sf.Alignment = StringAlignment.Center;
            g.DrawString("手掌（掌心朝右）", small, dimBrush, new PointF(405f, 555f), sf);
        }

        using (Pen bonePen = new Pen(BONE, 8f))
        using (SolidBrush jointBrush = new SolidBrush(JOINT))
        using (SolidBrush tipBrush = new SolidBrush(TIP))
        using (SolidBrush tipDragBrush = new SolidBrush(TIP_DRAG))
        using (Pen tipPen = new Pen(Color.FromArgb(122, 90, 0), 2f))
        using (SolidBrush textBrush = new SolidBrush(TEXT))
        {
            bonePen.StartCap = LineCap.Round;
            bonePen.EndCap = LineCap.Round;

            for (int i = 0; i < 5; i++)
            {
                PointF[] pts = FK(i, bend[i]);
                for (int k = 0; k < 3; k++)
                {
                    g.DrawLine(bonePen, pts[k], pts[k + 1]);
                }
                for (int k = 1; k < 3; k++)
                {
                    g.FillEllipse(jointBrush, pts[k].X - 8f, pts[k].Y - 8f, 16f, 16f);
                }
                bool dragging = (drag == i);
                float r = dragging ? 14f : 12f;
                g.FillEllipse(dragging ? tipDragBrush : tipBrush,
                    pts[3].X - r, pts[3].Y - r, r * 2f, r * 2f);
                g.DrawEllipse(tipPen, pts[3].X - r, pts[3].Y - r, r * 2f, r * 2f);

                float labelX = (i == 0) ? pts[3].X - 22f : pts[3].X + 22f;
                using (StringFormat sf = new StringFormat())
                {
                    sf.Alignment = (i == 0) ? StringAlignment.Far : StringAlignment.Near;
                    g.DrawString(
                        string.Format("{0} {1,3}%", FINGER_NAMES[i], (int)Math.Round(bend[i] * 100f)),
                        font, textBrush, new PointF(labelX, pts[3].Y - 22f), sf);
                }
            }
        }

        using (SolidBrush textBrush = new SolidBrush(TEXT))
        {
            g.DrawString(
                string.Format(
                    "UDP {0}:{1}  |  已发 {2} 包  |  {3} Hz  |  拖指尖弯曲，R 复位，Esc 退出",
                    host, port, sent, (int)rate),
                font, textBrush, new PointF(20f, 14f));
        }
    }

    // ---------- 发送 ----------

    float[] Snapshot()
    {
        lock (bendLock)
        {
            return (float[])bend.Clone();
        }
    }

    static float[] BuildQuaternions(float[] b)
    {
        float[] q = new float[64];   // 16 关节 × 4
        for (int j = 0; j < 16; j++)
        {
            q[j * 4 + 0] = 1f;       // 单位四元数 (w,x,y,z) = (1,0,0,0)
        }
        for (int i = 0; i < 5; i++)
        {
            float c = b[i];
            for (int seg = 0; seg < 3; seg++)
            {
                float deg = SEG_BEND[seg] * c;
                double half = deg * Math.PI / 360.0;
                int joint = BASE_JOINT[i] + seg;
                q[joint * 4 + 0] = (float)Math.Cos(half);
                q[joint * 4 + 1] = 0f;
                q[joint * 4 + 2] = (float)(-Math.Sin(half));
                q[joint * 4 + 3] = 0f;
            }
        }
        return q;
    }

    static byte[] BuildPacket(long seq, float[] q, int hand)
    {
        MemoryStream ms = new MemoryStream(272);
        BinaryWriter bw = new BinaryWriter(ms);
        bw.Write((byte)'E');
        bw.Write((byte)'I');
        bw.Write((byte)'G');
        bw.Write((byte)'V');
        bw.Write(VERSION);
        bw.Write((byte)hand);
        bw.Write((byte)0);
        bw.Write((uint)seq);
        bw.Write((uint)(DateTime.UtcNow.Ticks / TimeSpan.TicksPerMillisecond));
        for (int i = 0; i < 16; i++)
        {
            bw.Write(q[i * 4 + 0]);
            bw.Write(q[i * 4 + 1]);
            bw.Write(q[i * 4 + 2]);
            bw.Write(q[i * 4 + 3]);
        }
        bw.Flush();
        byte[] data = ms.ToArray();
        bw.Close();
        ms.Close();
        return data;
    }

    void SendLoop()
    {
        UdpClient udp;
        try
        {
            udp = new UdpClient();
        }
        catch
        {
            return;
        }

        IPEndPoint ep = new IPEndPoint(IPAddress.Parse(host), port);
        long seq = 0;
        double period = 1.0 / SEND_HZ;
        System.Diagnostics.Stopwatch sw = System.Diagnostics.Stopwatch.StartNew();

        while (running)
        {
            float[] b = Snapshot();
            byte[] packet = BuildPacket(seq, BuildQuaternions(b), hand);
            try
            {
                udp.Send(packet, packet.Length, ep);
                Interlocked.Increment(ref sent);
            }
            catch
            {
            }

            seq++;
            double target = seq * period;
            double elapsed = sw.Elapsed.TotalSeconds;
            if (target > elapsed)
            {
                Thread.Sleep((int)Math.Max(0, (target - elapsed) * 1000.0));
            }
        }

        try { udp.Close(); }
        catch { }
    }

    protected override void OnFormClosing(FormClosingEventArgs e)
    {
        running = false;
        try
        {
            if (sender != null && sender.IsAlive) sender.Join(200);
        }
        catch { }
        timer.Stop();
        base.OnFormClosing(e);
    }
}
