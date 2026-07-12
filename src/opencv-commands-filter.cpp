//
// Rebol/OpenCV extension — Image filtering, color conversion, edge detection,
// thresholding, and morphological operations
// ====================================
// Defines: bilateralFilter, blur, dilate, erode, filter2D, GaussianBlur,
// getGaborKernel, getStructuringElement, Laplacian, medianBlur,
// cvtColor, applyColorMap, Canny, threshold.
//

#include "opencv-commands-common.h"


//;-----------------------------------------------------------------------
//;- Image Filtering                                                      
//;- https://docs.opencv.org/4.6.0/d4/d86/group__imgproc__filter.html     
//;-----------------------------------------------------------------------

COMMAND cmd_bilateralFilter(RXIFRM *frm, void *ctx) {
	Mat *src          = ARG_Mat(1);
	Mat *dst          = ARG_Mat_As(2, src);
	int d             = ARG_Int(3);
	double sigmaColor = ARG_Double(4);
	double sigmaSpace = ARG_Double(5);
	int borderType    = ARG_BorderType(7);

	if(!src || !dst) return RXR_FALSE;

	EXCEPTION_TRY
	bilateralFilter(*src, *dst, d, sigmaColor, sigmaSpace, borderType);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}

COMMAND cmd_blur(RXIFRM *frm, void *ctx) {
	Mat *src          = ARG_Mat(1);
	Mat *dst          = ARG_Mat_As(2, src);
	Size ksize        = ARG_Size(3);
	Point anchor      = Point(-1, -1);
	int borderType    = ARG_BorderType(5);

	if(!src || !dst) return RXR_FALSE;

	EXCEPTION_TRY
	blur(*src, *dst, ksize, anchor, borderType);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}

COMMAND cmd_dilate(RXIFRM *frm, void *ctx) {
	Mat *src          = ARG_Mat(1);
	Mat *dst          = ARG_Mat_As(2, src);
	Mat *kernel       = ARG_Mat(3);
	Point anchor      = ARG_Point(4);
	int iterations    = ARG_Int(5);

	if(!src || !dst || !kernel) return RXR_FALSE;

	EXCEPTION_TRY
	dilate(*src, *dst, *kernel, anchor, iterations);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}

COMMAND cmd_erode(RXIFRM *frm, void *ctx) {
	Mat *src          = ARG_Mat(1);
	Mat *dst          = ARG_Mat_As(2, src);
	Mat *kernel       = ARG_Mat(3);
	Point anchor      = ARG_Point(4);
	int iterations    = ARG_Int(5);

	if(!src || !dst || !kernel) return RXR_FALSE;

	EXCEPTION_TRY
	erode(*src, *dst, *kernel, anchor, iterations);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}

COMMAND cmd_filter2D(RXIFRM *frm, void *ctx) {
	Mat *src          = ARG_Mat(1);
	Mat *dst          = ARG_Mat_As(2, src);
	int ddepth        = ARG_Int(3);
	Mat *kernel       = ARG_Mat(4);
	Point anchor      = ARG_Point(5);
	double delta      = ARG_Double(6);
	int borderType    = ARG_BorderType(8);

	if(!src || !dst || !kernel) return RXR_FALSE;

	EXCEPTION_TRY
	filter2D(*src, *dst, ddepth, *kernel, anchor, delta, borderType);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}

COMMAND cmd_GaussianBlur(RXIFRM *frm, void *ctx) {
	Mat *src          = ARG_Mat(1);
	Mat *dst          = ARG_Mat_As(2, src);
	Size ksize        = ARG_Size(3);
	double sigmaX     = ARG_Double(4);
	double sigmaY     = ARG_Double(5);
	int borderType    = ARG_BorderType(7);

	if (!dst || !src) return RXR_FALSE;

	EXCEPTION_TRY
	GaussianBlur(*src, *dst, ksize, sigmaX, sigmaY, borderType);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}

COMMAND cmd_getGaborKernel(RXIFRM *frm, void *ctx) {
	Size 	ksize  = ARG_Size(1);
	double 	sigma  = ARG_Double(2);
	double 	theta  = ARG_Double(3);
	double 	lambd  = ARG_Double(4);
	double 	gamma  = ARG_Double(5);
	double 	psi    = ARG_Double(6);
	int 	ktype  = ARG_Int(7);
	Mat    *kernel = new Mat();

	EXCEPTION_TRY
	*kernel = getGaborKernel(ksize, sigma, theta, lambd, gamma, psi, ktype);
	EXCEPTION_CATCH

	return initRXHandle(frm, 1, kernel, Handle_cvMat, NULL);
}

COMMAND cmd_getStructuringElement(RXIFRM *frm, void *ctx) {
	int shape     = ARG_Int(1);
	Size ksize    = ARG_Size(2);
	Point anchor  = ARG_Point(3);
	Mat *dst      = new Mat();

	EXCEPTION_TRY
	*dst = getStructuringElement(shape, ksize, anchor);
	EXCEPTION_CATCH

	return initRXHandle(frm, 1, dst, Handle_cvMat, NULL);
}

COMMAND cmd_Laplacian(RXIFRM *frm, void *ctx) {
	Mat *src       = ARG_Mat(1);
	Mat *dst       = ARG_Mat_As(2, src);
	int ddepth     = ARG_Int(3);
	int ksize      = ARG_Int(4);
	double scale   = ARG_Double(5);
	double delta   = ARG_Double(6);
	int borderType = BORDER_DEFAULT;

	if (!dst || !src) return RXR_FALSE;

	EXCEPTION_TRY
	Laplacian(*src, *dst, ddepth, ksize, scale, delta, borderType);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}

COMMAND cmd_medianBlur(RXIFRM *frm, void *ctx) {
	Mat *src       = ARG_Mat(1);
	Mat *dst       = ARG_Mat_As(2, src);
	int ksize      = ARG_Int(3);

	if (!dst || !src) return RXR_FALSE;

	EXCEPTION_TRY
	medianBlur(*src, *dst, ksize);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}


//;-----------------------------------------------------------------------
//;- Color Space Conversions                                              
//;- https://docs.opencv.org/4.6.0/d8/d01/group__imgproc__color__conversions.html
//;-----------------------------------------------------------------------

COMMAND cmd_cvtColor(RXIFRM *frm, void *ctx) {
	Mat *src       = ARG_Mat(1);
	Mat *dst       = ARG_Mat_As(2, src);
	int code       = ARG_Int(3);

	if (!src || !dst) return RXR_NONE;

	EXCEPTION_TRY
	cvtColor(*src, *dst, code);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}


//;-----------------------------------------------------------------------------------------------
//;- Color Maps                                                                                   
//;- https://docs.opencv.org/4.6.0/d3/d50/group__imgproc__colormap.html                           
//;-----------------------------------------------------------------------------------------------

COMMAND cmd_applyColorMap(RXIFRM *frm, void *ctx) {
	Mat *src       = ARG_Mat(1);
	Mat *dst       = ARG_Mat_As(2, src);
	int colormap   = ARG_Int(3);

	if (!src || !dst) return RXR_NONE;

	EXCEPTION_TRY
	applyColorMap(*src, *dst, colormap);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}


//;-----------------------------------------------------------------------------------------------
//;- Feature Detection                                                                            
//;- https://docs.opencv.org/4.6.0/dd/d1a/group__imgproc__feature.html                            
//;-----------------------------------------------------------------------------------------------

COMMAND cmd_Canny(RXIFRM *frm, void *ctx) {
	Mat *src       = ARG_Mat(1);
	Mat *dst       = ARG_Mat_As(2, src);
	double threshold1 = ARG_Double(3);
	double threshold2 = ARG_Double(4);

	if (!src || !dst) return RXR_NONE;

	EXCEPTION_TRY
	Canny(*src, *dst, threshold1, threshold2);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}


//;-----------------------------------------------------------------------
//;- Image Thresholding                                                   
//;- https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html     
//;-----------------------------------------------------------------------

COMMAND cmd_threshold(RXIFRM *frm, void *ctx) {
	Mat *src = ARG_Mat(1);
	Mat *dst = ARG_Mat_As(2, src);
	double thresh = ARG_Double(3);
	double maxval = ARG_Double(4);
	int    type   = ARG_Int(5);
	double result;

	if (!src || !dst) return RXR_NONE;

	EXCEPTION_TRY
	result = threshold(*src, *dst, thresh, maxval, type);
	EXCEPTION_CATCH

	if (type == THRESH_OTSU || type == THRESH_TRIANGLE){
		RXA_TYPE(frm, 1) = RXT_DECIMAL;
		RXA_DEC64(frm, 1) = result;
	} else {
		RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	}
	return RXR_VALUE;
}
