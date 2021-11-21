using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GoAcceleratorEngine
{
    public class ErrorInfo
    {
        private long code = -1;
        private string message,
            stack;
        readonly string[] keys = new string[] { "ErrorMessage", "ErrorNumber", "ErrorTrace" };
        public ErrorInfo(long code, string message, string stack="")
        {
            this.code = code;
            this.message = message;
            this.stack = stack;
        }
        public ErrorInfo(string info="")
        {
            if (0 < info.Length)
            {
                // replace all \r \n with space
                info = System.Text.RegularExpressions.Regex.Replace(info, "[\r\n][\r\n]*", " ");
                foreach (var s in info.Split(new char[] { '|' }))
                {
                    var a = s.Split(new char[] { ':' });
                    if (2 > a.Length) continue;
                    if (keys[0] == a[0]) message = a[1];
                    else if (keys[1] == a[0]) code = Convert.ToInt64(a[1]);
                    else if (keys[2] == a[0]) stack = a[1];
                }
            }
        }
        override public string ToString()
        {
            return string.Format("ErrorMessage:{0}|ErrorNumber:{1}|ErrorTrace:{2}", message, code, stack);
        }
        public long ErrorNumber { get { return code; } }
        public string ErrorMsg { get { return message; } }
        public string Stack { get { return stack; } }
    }
}
