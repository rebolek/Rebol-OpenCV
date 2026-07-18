//
// Rebol/OpenCV extension — High-level GUI and interaction
// ====================================
// Defines: startWindowThread, imshow, pollKey, waitKey, namedWindow,
// resizeWindow, moveWindow, getWindowImageSize, getWindowImagePos,
// getWindowProperty, setWindowProperty, setWindowTitle,
// destroyAllWindows, destroyWindow, createTrackbar,
// setTrackbarMax/Min/Pos, getTrackbarPos, selectROI, setMouseCallback.
//

#include "opencv-commands-common.h"


//;-----------------------------------------------------------------------
//;- High-level GUI                                                       
//;- https://docs.opencv.org/5.0.0/d7/dfc/group__highgui.html             
//;-----------------------------------------------------------------------

COMMAND cmd_startWindowThread(RXIFRM *frm, void *ctx) {
	startWindowThread();
	return RXR_UNSET;
}

COMMAND cmd_imshow(RXIFRM *frm, void *ctx) {
	Mat *image  = ARG_Mat(1);
	String name = (RXA_TYPE(frm, 3) == RXT_NONE) ? "Image" : ARG_String(3);

	if (image) {
		Size size = image->size();
		if (size.width == 0 || size.height == 0) return RXR_FALSE;
		EXCEPTION_TRY
		imshow(name, *image);
		EXCEPTION_CATCH
	} else if(ARG_Is_Image(1)) {
		RXIARG arg = RXA_ARG(frm, 1);
		Mat image(arg.height, arg.width, CV_8UC4, ((REBSER*)arg.series)->data);
		imshow(name, image);
	} else {
		return RXR_FALSE;
	}
	return RXR_TRUE;
}

COMMAND cmd_pollKey(RXIFRM *frm, void *ctx) {
	RXA_TYPE(frm, 1) = RXT_INTEGER;
	RXA_ARG(frm, 1).int64 = pollKey();
	return RXR_VALUE;
}

COMMAND cmd_waitKey(RXIFRM *frm, void *ctx) {
	RXA_ARG(frm, 1).int64 = waitKey(RXA_ARG(frm, 1).int32a);
	return RXR_VALUE;
}

COMMAND cmd_namedWindow(RXIFRM *frm, void *ctx) {
	namedWindow(ARG_String(1), WINDOW_NORMAL);
	pollKey(); // OpenCV bug on macOS... window would not resize without it!
	return RXR_UNSET;
}

COMMAND cmd_resizeWindow(RXIFRM *frm, void *ctx) {
	resizeWindow(ARG_String(1), PAIR_X(frm,2), PAIR_Y(frm,2));
	return RXR_UNSET;
}

COMMAND cmd_moveWindow(RXIFRM *frm, void *ctx) {
	moveWindow(ARG_String(1), PAIR_X(frm,2), PAIR_Y(frm,2));
	return RXR_UNSET;
}

COMMAND cmd_getWindowImageSize(RXIFRM *frm, void *ctx) {
	Rect rect = getWindowImageRect(ARG_String(1));
	RXA_TYPE(frm, 1) = RXT_PAIR;
	RXA_PAIR(frm,1).x = rect.width;
	RXA_PAIR(frm,1).y = rect.height;
	return RXR_VALUE;
}

COMMAND cmd_getWindowImagePos(RXIFRM *frm, void *ctx) {
	Rect rect = getWindowImageRect(ARG_String(1));
	RXA_TYPE(frm, 1) = RXT_PAIR;
	RXA_PAIR(frm,1).x = rect.x;
	RXA_PAIR(frm,1).y = rect.y;
	return RXR_VALUE;
}

COMMAND cmd_getWindowProperty(RXIFRM *frm, void *ctx) {
	RXA_TYPE(frm, 1) = RXT_DECIMAL;
	RXA_DEC64(frm, 1) = getWindowProperty(ARG_String(1), ARG_Int(2));
	return RXR_VALUE;
}

COMMAND cmd_setWindowProperty(RXIFRM *frm, void *ctx) {
	setWindowProperty(ARG_String(1), ARG_Int(2), ARG_Int(3));
	return RXR_UNSET;
}

COMMAND cmd_setWindowTitle(RXIFRM *frm, void *ctx) {
	setWindowTitle(ARG_String(1), ARG_String(2));
	return RXR_UNSET;
}

COMMAND cmd_destroyAllWindows(RXIFRM *frm, void *ctx) {
	destroyAllWindows();
	pollKey();
	return RXR_UNSET;
}

COMMAND cmd_destroyWindow(RXIFRM *frm, void *ctx) {
	destroyWindow(ARG_String(1));
	return RXR_UNSET;
}


//;-----------------------------------------------------------------------
//;- Trackbar                                                             
//;-----------------------------------------------------------------------

static void rebTrackbarCallback(int pos, void* userdata) {
	REBHOB* hob = (REBHOB*)userdata;
	CTX_TRACKBAR* bar = (CTX_TRACKBAR*)hob->data;
	if (!bar) {trace("null trackbar handle!"); return;}
	RXIARG *args = bar->args;
	RXICBI *cbi  = bar->cbi;
	if (!args || !cbi) {trace("!args || !cbi"); return;}

	RXI_COUNT(args) = 1;
	RXI_TYPE(args, 1) = RXT_INTEGER;
	args[1].int64 = pos;

	RL_CALLBACK(cbi);
}

COMMAND cmd_createTrackbar(RXIFRM *frm, void *ctx) {
	REBHOB* hob = RL_MAKE_HANDLE_CONTEXT(Handle_cvTrackbar);
	debug_print("new hob: %p handle: %p (cvTrackbar)\n", hob, hob->data);
	if (hob == NULL) return RXR_FALSE;

	CTX_TRACKBAR* bar = (CTX_TRACKBAR*)hob->data;
	bar->name   = new String(ARG_String(1));
	bar->window = new String(ARG_String(2));
	EXCEPTION_TRY
	if (ARG_Is_None(4)) {
		createTrackbar(*bar->name, *bar->window, &bar->value, ARG_Int(3));
	} else {
		bar->cbi    = (RXICBI *)MAKE_MEM(sizeof(RXICBI));
		bar->args   = (RXIARG *)MAKE_MEM(sizeof(RXIARG) * 4);

		CLEAR(bar->cbi,  sizeof(RXICBI));
		CLEAR(bar->args, sizeof(RXIARG) * 4);

		bar->cbi->obj  = (REBSER*)RXA_OBJECT(frm, 5);
		bar->cbi->word = RXA_WORD(frm, 6);
		bar->cbi->args = bar->args;

		createTrackbar(*bar->name, *bar->window, &bar->value, ARG_Int(3), rebTrackbarCallback, hob);
	}
	EXCEPTION_CATCH

	RXA_HANDLE(frm, 1) = hob;
	RXA_HANDLE_TYPE(frm, 1) = hob->sym;
	RXA_HANDLE_FLAGS(frm, 1) = hob->flags;
	RXA_TYPE(frm, 1) = RXT_HANDLE;

	return RXR_VALUE;
}

COMMAND cmd_setTrackbarMax(RXIFRM *frm, void *ctx) {
	CTX_TRACKBAR* bar = ARG_Trackbar(1);
	if (!bar) return RXR_FALSE;
	setTrackbarMax(*bar->name, *bar->window, ARG_Int(2));
	return RXR_VALUE;
}
COMMAND cmd_setTrackbarMin(RXIFRM *frm, void *ctx) {
	CTX_TRACKBAR* bar = ARG_Trackbar(1);
	if (!bar) return RXR_FALSE;
	setTrackbarMin(*bar->name, *bar->window, ARG_Int(2));
	return RXR_VALUE;
}
COMMAND cmd_setTrackbarPos(RXIFRM *frm, void *ctx) {
	CTX_TRACKBAR* bar = ARG_Trackbar(1);
	if (!bar) return RXR_FALSE;
	setTrackbarPos(*bar->name, *bar->window, ARG_Int(2));
	return RXR_VALUE;
}
COMMAND cmd_getTrackbarPos(RXIFRM *frm, void *ctx) {
	CTX_TRACKBAR* bar = ARG_Trackbar(1);
	if (!bar) return RXR_FALSE;
	RXA_TYPE(frm, 1) = RXT_INTEGER;
	RXA_ARG(frm, 1).int64 = getTrackbarPos(*bar->name, *bar->window);
	return RXR_VALUE;
}


//;-----------------------------------------------------------------------
//;- selectROI                                                            
//;-----------------------------------------------------------------------

COMMAND cmd_selectROI(RXIFRM *frm, void *ctx) {
	Mat *image  = ARG_Mat(1);
	Rect rect;

	if (image) {
		Size size = image->size();
		if (size.width == 0 || size.height == 0) return RXR_FALSE;
		EXCEPTION_TRY
		rect = selectROI(*image);
		EXCEPTION_CATCH
	} else if(ARG_Is_Image(1)) {
		RXIARG arg = RXA_ARG(frm, 1);
		Mat image(arg.height, arg.width, CV_8UC4, ((REBSER*)arg.series)->data);
		rect = selectROI(image);
	} else {
		return RXR_FALSE;
	}

	REBSER *blk = (REBSER*)RL_MAKE_BLOCK(2);
	RXIARG val;

	val.pair.x = (float)rect.x;
	val.pair.y = (float)rect.y;
	RL_SET_VALUE(blk, 0, val, RXT_PAIR);
	val.pair.x = (float)rect.width;
	val.pair.y = (float)rect.height;
	RL_SET_VALUE(blk, 1, val, RXT_PAIR);

	RXA_TYPE(frm, 1) = RXT_BLOCK;
	RXA_SERIES(frm, 1) = blk;
	RXA_INDEX(frm, 1) = 0;
	return RXR_VALUE;
}


//;-----------------------------------------------------------------------
//;- Mouse callbacks                                                      
//;-----------------------------------------------------------------------

static void rebMouseCallback(int event, int x, int y, int flags, void* userdata) {
	REBHOB* hob = (REBHOB*)userdata;
    CTX_MOUSECALLBACK* mcb = (CTX_MOUSECALLBACK*)hob->data;
	if (!mcb) {trace("null mouseCallback handle!"); return;}
	RXIARG *args = mcb->args;
	RXICBI *cbi  = mcb->cbi;
	if (!args || !cbi) {trace("!args || !cbi"); return;}

	args[1].int64 = event;
	args[2].int64 = mcb->x = x;
	args[3].int64 = mcb->y = y;
	args[4].int64 = mcb->flags = flags;

	RL_CALLBACK(cbi);
}

COMMAND cmd_setMouseCallback(RXIFRM *frm, void *ctx) {
	// Removing the callback needs no handle context; allocate only when
	// installing one, otherwise the abandoned hob would leak.
	if (ARG_Is_None(3)) {
		setMouseCallback(ARG_String(1), NULL);
		return RXR_UNSET;
	}

	REBHOB* hob = RL_MAKE_HANDLE_CONTEXT(Handle_cvMouseCallback);
	if (hob == NULL) return RXR_FALSE;

	CTX_MOUSECALLBACK* mcb = (CTX_MOUSECALLBACK*)hob->data;
	mcb->window = new String(ARG_String(1));
	mcb->cbi    = (RXICBI*)MAKE_MEM(sizeof(RXICBI));
	mcb->args   = (RXIARG*)MAKE_MEM(sizeof(RXIARG) * 5);
	CLEAR(mcb->cbi,  sizeof(RXICBI));
	CLEAR(mcb->args, sizeof(RXIARG) * 5);
	mcb->cbi->obj  = (REBSER*)RXA_OBJECT(frm, 2);
	mcb->cbi->word = RXA_WORD(frm, 3);
	mcb->cbi->args = mcb->args;

	RXI_COUNT(mcb->args) = 4;
	RXI_TYPE(mcb->args, 1) = RXT_INTEGER;
	RXI_TYPE(mcb->args, 2) = RXT_INTEGER;
	RXI_TYPE(mcb->args, 3) = RXT_INTEGER;
	RXI_TYPE(mcb->args, 4) = RXT_INTEGER;

	setMouseCallback(*mcb->window, rebMouseCallback, hob);

	RXA_HANDLE(frm, 1) = hob;
	RXA_HANDLE_TYPE(frm, 1) = hob->sym;
	RXA_HANDLE_FLAGS(frm, 1) = hob->flags;
	RXA_TYPE(frm, 1) = RXT_HANDLE;
	return RXR_VALUE;
}
