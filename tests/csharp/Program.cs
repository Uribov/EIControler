using System;
using EIGlove;

static class Program
{
    static int fails;

    static void Check(string name, bool ok, string detail = "")
    {
        Console.WriteLine($"  [{(ok ? "PASS" : "FAIL")}] {name} {detail}");
        if (!ok) fails++;
    }

    static byte[] BuildPacket(ushort version, int hand, float value)
    {
        var buf = new byte[EIProtocol.PacketSize];
        buf[0] = (byte)'E'; buf[1] = (byte)'I'; buf[2] = (byte)'G'; buf[3] = (byte)'V';
        buf[4] = (byte)(version & 0xFF);
        buf[5] = (byte)(version >> 8);
        buf[6] = (byte)hand;
        for (int i = 0; i < EIProtocol.JointCount; i++)
        {
            int o = 16 + i * 16;
            BitConverter.GetBytes(1.0f).CopyTo(buf, o);
            BitConverter.GetBytes(value).CopyTo(buf, o + 4);
            BitConverter.GetBytes(0.0f).CopyTo(buf, o + 8);
            BitConverter.GetBytes(0.0f).CopyTo(buf, o + 12);
        }
        return buf;
    }

    static void Main()
    {
        Console.WriteLine("== 协议解析 ==");

        byte[] good = BuildPacket(1, 0, 0.5f);
        Check("正常包能解析", EIProtocol.TryParse(good, out var q));
        Check("关节数 16", q != null && q.Length == 64);
        Check("w 读到 1.0", q != null && Math.Abs(q[0] - 1.0f) < 1e-6);
        Check("x 读到 0.5", q != null && Math.Abs(q[1] - 0.5f) < 1e-6);
        Check("手别字段", EIProtocol.HandOf(good) == 0);

        Check("短包被拒", !EIProtocol.TryParse(new byte[100], out _));
        Check("空包被拒", !EIProtocol.TryParse(null, out _));

        byte[] badMagic = BuildPacket(1, 0, 0);
        badMagic[0] = (byte)'X';
        Check("错魔数被拒", !EIProtocol.TryParse(badMagic, out _));

        byte[] badVer = BuildPacket(2, 0, 0);
        Check("错版本被拒", !EIProtocol.TryParse(badVer, out _));

        byte[] left = BuildPacket(1, 1, 0);
        Check("左手包手别为 1", EIProtocol.HandOf(left) == 1);

        Console.WriteLine();
        Console.WriteLine(fails == 0 ? "全部通过" : $"有 {fails} 项失败");
        Environment.Exit(fails == 0 ? 0 : 1);
    }
}
