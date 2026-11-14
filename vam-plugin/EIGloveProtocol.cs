// EI Controller 手套与 VaM 插件之间的 UDP 协议解析。
//
// 这个文件不依赖 Unity、不依赖 VaM，是纯 .NET 代码，可以在 Linux 上单独编译测试。
// 插件主体（EIGloveDriver.cs）用它解析数据包。

using System;

namespace EIGlove
{
    public static class EIProtocol
    {
        public const int PacketSize = 272;
        public const int JointCount = 16;

        /// <summary>
        /// 解析一个 UDP 包。成功时 out 数组长度为 JointCount * 4，顺序是每关节 w,x,y,z。
        /// </summary>
        public static bool TryParse(byte[] data, out float[] quaternions)
        {
            quaternions = null;

            if (data == null || data.Length != PacketSize)
            {
                return false;
            }

            // 魔数 EIGV
            if (data[0] != (byte)'E' || data[1] != (byte)'I'
                || data[2] != (byte)'G' || data[3] != (byte)'V')
            {
                return false;
            }

            // 版本号，不匹配就不认
            ushort version = (ushort)(data[4] | (data[5] << 8));
            if (version != 1)
            {
                return false;
            }

            var outQ = new float[JointCount * 4];
            for (int i = 0; i < JointCount; i++)
            {
                int o = 16 + i * 16;
                outQ[i * 4 + 0] = BitConverter.ToSingle(data, o);      // w
                outQ[i * 4 + 1] = BitConverter.ToSingle(data, o + 4);  // x
                outQ[i * 4 + 2] = BitConverter.ToSingle(data, o + 8);  // y
                outQ[i * 4 + 3] = BitConverter.ToSingle(data, o + 12); // z
            }

            quaternions = outQ;
            return true;
        }

        /// <summary>包里的手别字段：0 右手，1 左手</summary>
        public static int HandOf(byte[] data)
        {
            if (data == null || data.Length < 8)
            {
                return 0;
            }
            return data[6];
        }
    }
}
