using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace GocatorGUI {
    public class CWrapper {
        /*
        [DllImport("Gocator.dll")]
        private static extern IntPtr CreateGocator();

        [DllImport("Gocator.dll")]
        private static extern void DeleteGocator(IntPtr class_pointer);

        [DllImport("Gocator.dll")]
        private static extern Error Gocator_Init(IntPtr class_pointer);

        [DllImport("Gocator.dll")]
        private static extern Error Gocator_Start(IntPtr class_pointer);

        [DllImport("Gocator.dll")]
        private static extern Error Gocator_Stop(IntPtr class_pointer);

        [DllImport("Gocator.dll")]
        private static extern Error Gocator_SetParameter(IntPtr class_pointer, ParameterType name, void* value);

        private readonly IntPtr _pointerclass;

        public CWrapper() {
            _pointerclass = CreateGocator();
        }

        public Error Gocator_Init() {
            return Gocator_Init(_pointerclass);
        }

        public Error Gocator_Start() {
            return Gocator_Start(_pointerclass);
        }

        public Error Gocator_Stop() {
            return Gocator_Stop(_pointerclass);
        }

        public Error Gocator_SetParameter(ParameterType name, void* value) {
            return Gocator_SetParameter(_pointerclass, name, value);
        }*/
    }
}