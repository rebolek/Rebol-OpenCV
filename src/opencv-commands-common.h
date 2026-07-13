//
// Rebol/OpenCV extension — shared definitions for all command modules
// ====================================
// Included by every split opencv-commands-*.cpp file.
// Contains macros, type mappings, and helper declarations.
//

#pragma once

#include "opencv2/opencv.hpp"

extern "C" {
	#include "opencv-rebol-extension.h"
}

// Temporary buffer used to pass exception messages to the Rebol side.
// Must be a global because the error string could be freed before Rebol reads it.
extern char* err_buff[255];

//----------------------------------------------------------------------
// Exception handling — wraps each OpenCV call in try/catch and returns
// a Rebol error on cv::Exception.
//----------------------------------------------------------------------
#define EXCEPTION_TRY   try {
#define EXCEPTION_CATCH } catch( cv::Exception& e ) {\
	snprintf((char*)err_buff, 254,"%s", e.err.c_str());\
	RXA_SERIES(frm, 1) = (void*)err_buff;\
	return RXR_ERROR;}

#define COMMAND           extern "C" int
#define PAIR_X(frm, n)    (int)RXA_PAIR(frm,n).x
#define PAIR_Y(frm, n)    (int)RXA_PAIR(frm,n).y

//----------------------------------------------------------------------
// Argument type-checking helpers
//----------------------------------------------------------------------
#define FRM_IS_HANDLE(n, t)     (RXA_TYPE(frm,n) == RXT_HANDLE && RXA_HANDLE_TYPE(frm, n) == t && IS_USED_HOB(RXA_HANDLE_CONTEXT(frm, n)))
#define ARG_Is_Mat(n)           FRM_IS_HANDLE(n, Handle_cvMat)
#define ARG_Is_VideoCapture(n)  FRM_IS_HANDLE(n, Handle_cvVideoCapture)
#define ARG_Is_VideoWriter(n)   FRM_IS_HANDLE(n, Handle_cvVideoWriter)
#define ARG_Is_Trackbar(n)      FRM_IS_HANDLE(n, Handle_cvTrackbar)
#define ARG_Is_Image(n)         (RXA_TYPE(frm,n) == RXT_IMAGE)
#define ARG_Is_Pair(n)          (RXA_TYPE(frm,n) == RXT_PAIR)
#define ARG_Is_Integer(n)       (RXA_TYPE(frm,n) == RXT_INTEGER)
#define ARG_Is_None(n)          (RXA_TYPE(frm,n) == RXT_NONE)
#define ARG_Is_Not_None(n)      (RXA_TYPE(frm,n) != RXT_NONE)
#define ARG_Is_Block(n)         (RXA_TYPE(frm,n) == RXT_BLOCK)
#define ARG_Is_File(n)          (RXA_TYPE(frm,n) == RXT_FILE)

//----------------------------------------------------------------------
// Argument extraction helpers
//----------------------------------------------------------------------
#define ARG_Mat(n)              (ARG_Is_Mat(n) ? (Mat*)(RXA_HANDLE_CONTEXT(frm, n)->handle) : NULL)
#define ARG_VideoCapture(n)     (VideoCapture*)(RXA_HANDLE_CONTEXT(frm, n)->handle)
#define ARG_VideoWriter(n)      (VideoWriter*)(RXA_HANDLE_CONTEXT(frm, n)->handle)
#define ARG_Trackbar(n)         (ARG_Is_Trackbar(n) ? (CTX_TRACKBAR*)(RXA_HANDLE_CONTEXT(frm, n)->handle) : NULL)
// type_words lists type symbols in fixed depth-then-channel order (see type-words:
// block in opencv-rebol-extension.r3): ordinal 0..31 encodes depth + (channels-1)*8.
// That stride-8 ordinal is NOT a real OpenCV type value once CV_CN_SHIFT != 3 (OpenCV 5
// uses 5), so it must be decoded and rebuilt via CV_MAKETYPE rather than used directly.
static inline int argMatType(RXIFRM *frm, int n) {
	if (RXA_TYPE(frm,n) == RXT_INTEGER) return RXA_INT32(frm,n);
	int ordinal = RL_FIND_WORD(type_words, RXA_INT32(frm,n)) - W_TYPE_CV_8UC1;
	return CV_MAKETYPE(ordinal & 7, (ordinal >> 3) + 1);
}
#define ARG_MatType(n)          argMatType(frm, n)
#define ARG_Double(n)           (RXA_TYPE(frm,n) == RXT_DECIMAL ? RXA_DEC64(frm,n) : (double)RXA_INT64(frm,n))
#define ARG_Int(n)              (RXA_TYPE(frm,n) == RXT_INTEGER ? RXA_INT32(frm,n) : (int)RXA_DEC64(frm,n))
#define ARG_Size(n)             (RXA_TYPE(frm,n) == RXT_PAIR ? Size(PAIR_X(frm,n), PAIR_Y(frm,n)) : Size(RXA_INT32(frm,n), RXA_INT32(frm,n)));
#define ARG_Point(n)            (RXA_TYPE(frm,n) == RXT_PAIR ? Point(PAIR_X(frm,n), PAIR_Y(frm,n)) : Point(RXA_INT32(frm,n), RXA_INT32(frm,n)));
// Extract a cv::String from a Rebol string argument, converting UCS-2 to UTF-8 if needed.
static inline cv::String argString(RXIFRM *frm, int n) {
    REBSER* ser = (REBSER*)RXA_ARG(frm, n).series;
    if (SERIES_WIDE(ser) == 1) {
        return cv::String((const char*)SERIES_DATA(ser), SERIES_TAIL(ser));
    }
    REBSER* utf8 = RL_ENCODE_UTF8_STRING(SERIES_DATA(ser), SERIES_TAIL(ser), TRUE, 0);
    return cv::String((const char*)SERIES_DATA(utf8), SERIES_TAIL(utf8));
}
#define ARG_String(n)           argString(frm, n)
#define ARG_BorderType(n)       (RXA_TYPE(frm, n) == RXT_INTEGER ? RXA_INT32(frm, n) : BORDER_DEFAULT)

// Special Mat initialization: if arg[n] is none, create a new Mat with same size/type as m
#define ARG_Mat_As(n,m)         (ARG_Is_Mat(n)\
								? (Mat*)(RXA_HANDLE_CONTEXT(frm, n)->handle)\
								: (m ? initRXHandle(frm,n,new Mat(m->size(),m->type()),Handle_cvMat, NULL),(Mat*)(RXA_HANDLE_CONTEXT(frm, n)->handle) : NULL))

//----------------------------------------------------------------------
// String builder — appends formatted text to a Rebol series
//----------------------------------------------------------------------
#define APPEND_STRING(str, ...) \
	len = snprintf(NULL,0,__VA_ARGS__);\
	if (len > SERIES_REST(str)-SERIES_LEN(str)) {\
		RL_EXPAND_SERIES(str, SERIES_TAIL(str), len);\
		SERIES_TAIL(str) -= len;\
	}\
	len = snprintf( \
		SERIES_TEXT(str)+SERIES_TAIL(str),\
		SERIES_REST(str)-SERIES_TAIL(str),\
		__VA_ARGS__\
	);\
	SERIES_TAIL(str) += len;

//----------------------------------------------------------------------
// Mat property IDs (used by cmd_get_property and cvMat_get_path)
//----------------------------------------------------------------------
enum MatProperties {
	MAT_SIZE = 1,
	MAT_TYPE,
	MAT_DEPTH,
	MAT_CHANNELS,
	MAT_BINARY,
	MAT_IMAGE,
	MAT_VECTOR,
};

// Per-channel byte size indexed by OpenCV depth (CV_8U, CV_8S, CV_16U, ...).
extern unsigned char depthByteSize[8];
// Total per-element byte size (all channels) for a real OpenCV type value.
// Computed from depth/channel count rather than indexed by raw type, since the
// channel stride in `type` (CV_CN_SHIFT) differs between OpenCV versions.
static inline int elementSizeForType(int type) {
	return depthByteSize[CV_MAT_DEPTH(type)] * CV_MAT_CN(type);
}

// Rebol vector type → OpenCV depth mapping.
extern int vecType2cvType[12];

//----------------------------------------------------------------------
// Handle type IDs (defined in opencv-rebol-extension.c)
//----------------------------------------------------------------------
using namespace cv;
using namespace std;

extern "C" {
	extern REBCNT Handle_cvVideoCapture;
	extern REBCNT Handle_cvVideoWriter;
	extern REBCNT Handle_cvMat;
	extern REBCNT Handle_cvTrackbar;
	extern REBCNT Handle_cvMouseCallback;

	//--------------------------------------------------------------
	// Helper functions (defined in opencv-commands-matrix.cpp)
	//--------------------------------------------------------------
	int initRXHandleArg(RXIARG* val, void* handle, REBCNT type, REBSER* ser);
	int initRXHandle(RXIFRM *frm, int index, void* handle, REBCNT type, REBSER* ser);
	Mat* new_Mat_From_Image_Arg(RXIFRM *frm, int index);
	REBSER* new_Reb_Vector(REBCNT size, int depth);
	void mat2ser(Mat *mat, REBSER *bin, RXIARG *arg);
	int Common_mold(REBHOB *hob, REBSER *str);

	//--------------------------------------------------------------
	// Handle lifecycle callbacks (referenced from opencv-rebol-extension.c)
	//--------------------------------------------------------------
	void* cvMat_free(void* cls);
	int cvMat_get_path(REBHOB *hob, REBCNT word, REBCNT *type, RXIARG *arg);
	void* cvVideoCapture_free(void* cls);
	int cvVideoCapture_get_path(REBHOB *hob, REBCNT word, REBCNT *type, RXIARG *arg);
	void* cvMouseCallback_free(void* cls);
	int cvMouseCallback_get_path(REBHOB *hob, REBCNT word, REBCNT *type, RXIARG *arg);
	void* releaseVideoWriter(void* cls);
	void* releaseTrackbar(void* cls);
}
