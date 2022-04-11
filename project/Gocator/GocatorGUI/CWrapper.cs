﻿using System;
using System.Runtime.InteropServices;
using System.Text;

namespace GocatorGUI {
    public class CWrapper {

        [DllImport("Gocator.dll")]
        private static extern IntPtr CreateGocatorManager();

        [DllImport("Gocator.dll")]
        private static extern bool GocatorManager_ServerStart(IntPtr class_pointer);

        [DllImport("Gocator.dll")]
        private static extern void GocatorManager_SetParameter(IntPtr class_pointer, StringBuilder str, int strlen, string param, int type);

        [DllImport("Gocator.dll")]
        private static extern void GocatorManager_Init(IntPtr class_pointer, StringBuilder str, int strlen);

        [DllImport("Gocator.dll")]
        private static extern void GocatorManager_LoadPointCloud(IntPtr class_pointer, StringBuilder str, int strlen, string strfilename);

        [DllImport("Gocator.dll")]
        private static extern void GocatorManager_StartAcquisition(IntPtr class_pointer, StringBuilder str, int strlen, int type, bool checkSavePCD);

        [DllImport("Gocator.dll")]
        private static extern void GocatorManager_StopAcquisition(IntPtr class_pointer, StringBuilder str, int strlen);
        
        [DllImport("Gocator.dll")]
        private static extern bool GocatorManager_FileAnalysis(IntPtr class_pointer, int type, bool checkSavePCD);

        private readonly IntPtr _pointerclass;

        public CWrapper() {
            _pointerclass = CreateGocatorManager();
        }

        public bool GocatorManager_ServerStart() {
            return GocatorManager_ServerStart(_pointerclass);
        }

        public void GocatorManager_SetParameter(StringBuilder str, int strlen, string param, int type) {
            GocatorManager_SetParameter(_pointerclass, str, strlen, param, type);
        }

        public void GocatorManager_Init(StringBuilder str, int strlen) {
            GocatorManager_Init(_pointerclass, str, strlen);
        }

        public void GocatorManager_LoadPointCloud(StringBuilder str, int strlen, string strfilename) {
            GocatorManager_LoadPointCloud(_pointerclass, str, strlen, strfilename);
        }

        public void GocatorManager_StartAcquisition(StringBuilder str, int strlen, int type, bool checkSavePCD) {
            GocatorManager_StartAcquisition(_pointerclass, str, strlen, type, checkSavePCD);
        }

        public void GocatorManager_StopAcquisition(StringBuilder str, int strlen) {
            GocatorManager_StopAcquisition(_pointerclass, str, strlen);
        }

        public bool GocatorManager_FileAnalysis(int type, bool checkSavePCL) {
            return GocatorManager_FileAnalysis(_pointerclass, type, checkSavePCL);
        }
    }
}