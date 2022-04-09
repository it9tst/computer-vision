using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace GocatorGUI {
    public class CWrapper {

        [DllImport("Gocator.dll")]
        private static extern IntPtr CreateGocatorManager();

        [DllImport("Gocator.dll")]
        private static extern bool GocatorManager_ServerStart(IntPtr class_pointer);

        [DllImport("Gocator.dll")]
        private static extern bool GocatorManager_SetParameter(IntPtr class_pointer, string param, int type);

        [DllImport("Gocator.dll")]
        private static extern bool GocatorManager_Init(IntPtr class_pointer);

        [DllImport("Gocator.dll")]
        private static extern bool GocatorManager_LoadPointCloud(IntPtr class_pointer, string strfilename);

        [DllImport("Gocator.dll")]
        private static extern bool GocatorManager_FileAnalysis(IntPtr class_pointer, int type);

        [DllImport("Gocator.dll")]
        private static extern bool GocatorManager_StartAcquisition(IntPtr class_pointer, int type);

        [DllImport("Gocator.dll")]
        private static extern bool GocatorManager_StopAcquisition(IntPtr class_pointer);

        private readonly IntPtr _pointerclass;

        public CWrapper() {
            _pointerclass = CreateGocatorManager();
        }

        public bool GocatorManager_ServerStart() {
            return GocatorManager_ServerStart(_pointerclass);
        }

        public bool GocatorManager_SetParameter(string param, int type) {
            return GocatorManager_SetParameter(_pointerclass, param, type);
        }

        public bool GocatorManager_Init() {
            return GocatorManager_Init(_pointerclass);
        }

        public bool GocatorManager_LoadPointCloud(string strfilename) {
            return GocatorManager_LoadPointCloud(_pointerclass, strfilename);
        }

        public bool GocatorManager_FileAnalysis(int type) {
            return GocatorManager_FileAnalysis(_pointerclass, type);
        }

        public bool GocatorManager_StartAcquisition(int type) {
            return GocatorManager_StartAcquisition(_pointerclass, type);
        }

        public bool GocatorManager_StopAcquisition() {
            return GocatorManager_StopAcquisition(_pointerclass);
        }
    }
}