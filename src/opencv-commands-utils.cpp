//
// Rebol/OpenCV extension — Utilities, QR codes, and test command
// ====================================
// Defines: cmd_test, getTickCount, getTickFrequency, getNumThreads,
// getVersionString, getBuildInformation, useOptimized, setUseOptimized,
// qrcode-encode, qrcode-decode.
//

#include "opencv-commands-common.h"


//;-----------------------------------------------------------------------
//;- Test command (opens camera, records a short video)                   
//;-----------------------------------------------------------------------

COMMAND cmd_test(RXIFRM *frm, void *ctx) {
	Mat src;
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    cap >> src;
    if (src.empty()) {
        cerr << "ERROR! blank frame grabbed\n";
        return -1;
    }
    bool isColor = (src.type() == CV_8UC3);
    VideoWriter writer;
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    double fps = 25.0;
    string filename = "./live.avi";
    writer.open(filename, codec, fps, src.size(), isColor);
    if (!writer.isOpened()) {
        cerr << "Could not open the output video file for write\n";
        return -1;
    }
    cout << "Writing videofile: " << filename << endl
         << "Press any key to terminate" << endl;
    for (;;)
    {
        if (!cap.read(src)) {
            cerr << "ERROR! blank frame grabbed\n";
            continue;
        }
        writer.write(src);
        imshow("Live", src);
        if (waitKey(5) >= 0)
            break;
    }
	return RXR_UNSET;
}


//;-----------------------------------------------------------------------
//;- Utilities                                                            
//;-----------------------------------------------------------------------

COMMAND cmd_getTickCount(RXIFRM *frm, void *ctx) {
	RXA_TYPE(frm, 1) = RXT_INTEGER;
	RXA_INT64(frm, 1) = getTickCount();
	return RXR_VALUE;
}
COMMAND cmd_getTickFrequency(RXIFRM *frm, void *ctx) {
	RXA_TYPE(frm, 1) = RXT_DECIMAL;
	RXA_DEC64(frm, 1) = getTickFrequency();
	return RXR_VALUE;
}
COMMAND cmd_getNumThreads(RXIFRM *frm, void *ctx) {
	RXA_TYPE(frm, 1) = RXT_INTEGER;
	RXA_INT64(frm, 1) = getNumThreads();
	return RXR_VALUE;
}
COMMAND cmd_getVersionString(RXIFRM *frm, void *ctx) {
	String version = getVersionString();
	int len = version.length();
	REBSER *ser = (REBSER *)RL_MAKE_STRING(len , FALSE);
	memcpy(ser->data, version.c_str(),  len);
	SERIES_TAIL(ser) = len;
	RXA_TYPE  (frm, 1) = RXT_STRING;
	RXA_SERIES(frm, 1) = ser;
	RXA_INDEX (frm, 1) = 0;
	return RXR_VALUE;
}
COMMAND cmd_getBuildInformation(RXIFRM *frm, void *ctx) {
	String str = getBuildInformation();
	int len = str.length();
	REBSER *ser = (REBSER *)RL_MAKE_STRING(len , FALSE);
	memcpy(ser->data, str.c_str(),  len);
	SERIES_TAIL(ser) = len;
	RXA_TYPE  (frm, 1) = RXT_STRING;
	RXA_SERIES(frm, 1) = ser;
	RXA_INDEX (frm, 1) = 0;
	return RXR_VALUE;
}
COMMAND cmd_useOptimized(RXIFRM *frm, void *ctx) {
	RXA_TYPE(frm, 1) = RXT_LOGIC;
	RXA_LOGIC(frm, 1) = useOptimized();
	return RXR_VALUE;
}
COMMAND cmd_setUseOptimized(RXIFRM *frm, void *ctx) {
	setUseOptimized(RXA_LOGIC(frm, 1));
	return RXR_VALUE;
}


//;-----------------------------------------------------------------------
//;- QR code encode/decode                                                
//;-----------------------------------------------------------------------

COMMAND cmd_qrcode_encode(RXIFRM *frm, void *ctx) {
	Mat *result;
	String text = ARG_String(1);

	result = new Mat();
	if (RXR_VALUE != initRXHandle(frm, 1, result, Handle_cvMat, NULL))
		return RXR_FALSE;

    QRCodeEncoder::Params params;
    if (RXA_TYPE(frm, 3) == RXT_INTEGER) params.version = RXA_INT32(frm, 3);
    if (RXA_TYPE(frm, 5) == RXT_INTEGER) params.mode = (QRCodeEncoder::EncodeMode)RXA_INT32(frm, 5);
    if (RXA_TYPE(frm, 7) == RXT_INTEGER) params.correction_level = (QRCodeEncoder::CorrectionLevel)RXA_INT32(frm, 7);

    EXCEPTION_TRY
    Ptr<QRCodeEncoder> encoder = QRCodeEncoder::create(params);
    encoder->encode(text, *result);
    EXCEPTION_CATCH
    return RXR_VALUE;
}

COMMAND cmd_qrcode_decode(RXIFRM *frm, void *ctx) {
	Mat *img = ARG_Mat(1);
	String str;
	vector<Point> corners;
	QRCodeDetector qrcode;

	EXCEPTION_TRY
	if (img == NULL) {
		Mat tmp;
		if (ARG_Is_Image(1)) {
			RXIARG arg = RXA_ARG(frm, 1);
			tmp = Mat(arg.height, arg.width, CV_8UC4);
			tmp.data = ((REBSER*)arg.series)->data;
		}
		else if (ARG_Is_File(1)) {
			String file = ARG_String(1);
			tmp = imread(file);
			if (tmp.empty()) return RXR_NONE;
		}
		else return RXR_NONE;
		str = qrcode.detectAndDecode(tmp, corners);
	} else {
		str = qrcode.detectAndDecode(*img, corners);
	}
	EXCEPTION_CATCH

	int len = str.length();
	if (len == 0) return RXR_NONE;

	REBSER *ser = (REBSER *)RL_MAKE_STRING(len , FALSE);
	memcpy(ser->data, str.c_str(),  len);
	SERIES_TAIL(ser) = len;
	RXA_TYPE  (frm, 1) = RXT_STRING;
	RXA_SERIES(frm, 1) = ser;
	RXA_INDEX (frm, 1) = 0;
	return RXR_VALUE;
}
