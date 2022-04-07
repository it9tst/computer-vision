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
        private static extern bool GocatorManager_SetParameter(IntPtr class_pointer);

        [DllImport("Gocator.dll")]
        private static extern bool GocatorManager_Init(IntPtr class_pointer);

        [DllImport("Gocator.dll")]
        private static extern bool GocatorManager_LoadPointCloud(IntPtr class_pointer, string strfilename);

        [DllImport("Gocator.dll")]
        private static extern bool GocatorManager_OfflineAnalysis(IntPtr class_pointer);

        private readonly IntPtr _pointerclass;

        public CWrapper() {
            _pointerclass = CreateGocatorManager();
        }

        public bool GocatorManager_SetParameter() {
            return GocatorManager_SetParameter(_pointerclass);
        }

        public bool GocatorManager_Init() {
            return GocatorManager_Init(_pointerclass);
        }

        public bool GocatorManager_LoadPointCloud(string strfilename) {
            return GocatorManager_LoadPointCloud(_pointerclass, strfilename);
        }

        public bool GocatorManager_OfflineAnalysis() {
            return GocatorManager_OfflineAnalysis(_pointerclass);
        }



        /*
        [DllImport("Gocator.dll")]
        private static extern Error Gocator_Start(IntPtr class_pointer);

        [DllImport("Gocator.dll")]
        private static extern Error Gocator_Stop(IntPtr class_pointer);

        

        public Error Gocator_Start() {
            return Gocator_Start(_pointerclass);
        }

        public Error Gocator_Stop() {
            return Gocator_Stop(_pointerclass);
        }

        
        */
    }
}