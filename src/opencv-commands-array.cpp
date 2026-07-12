//
// Rebol/OpenCV extension — Array operations and geometric transforms
// ====================================
// Defines: resize, add, subtract, multiply, divide, absdiff, addWeighted,
// bitwise_and/or/xor/not, convertScaleAbs, flip, invert, max, min, mean,
// minMaxLoc, normalize, transform, convertTo.
//

#include "opencv-commands-common.h"


//;-----------------------------------------------------------------------
//;- Geometric Image Transformations                                      
//;- https://docs.opencv.org/4.6.0/da/d54/group__imgproc__transform.html  
//;-----------------------------------------------------------------------

COMMAND cmd_resize(RXIFRM *frm, void *ctx) {
	Mat *src;
	Mat *dst;
	Size size;
	int interpolation = INTER_LINEAR;
	bool newHandle = FALSE;

	if (ARG_Is_Mat(1) && ARG_Mat(1)) {
		src = ARG_Mat(1);
	} else if (ARG_Is_Image(1)) {
		src = new_Mat_From_Image_Arg(frm, 1);
	} else return RXR_NONE;

	if (ARG_Is_Mat(4) && ARG_Mat(4)) {
		dst = ARG_Mat(4);
	} else {
		newHandle = TRUE;
		dst = new Mat();
	}

	if (ARG_Is_Pair(2)) {
		size = ARG_Size(2);
	} else {
		double scale = RXA_DEC64(frm, 2);
		size = src->size();
		size.width *= scale;
		size.height *= scale;
	}

	if (ARG_Is_Integer(6)) {
		interpolation = ARG_Int(6);
	}

	EXCEPTION_TRY
	resize(*src, *dst, size, 0, 0, interpolation);
	EXCEPTION_CATCH

	if (ARG_Is_Image(1)) {
		delete src;
	}

	if (newHandle) {
		return initRXHandle(frm, 1, dst, Handle_cvMat, NULL);
	}
	else {
		RXA_ARG(frm, 1) = RXA_ARG(frm, 4);
		return RXR_VALUE;
	}
}


//;-----------------------------------------------------------------------
//;- Operations on arrays                                                 
//;- https://docs.opencv.org/4.6.0/d2/de8/group__core__array.html         
//;-----------------------------------------------------------------------

enum  MatMathOp {
  BITWISE_AND = 0,
  BITWISE_OR  = 1,
  BITWISE_XOR = 2,
  MATH_ADD,
  MATH_SUBTRACT,
  MATH_MULTIPLY = 10,
  MATH_DIVIDE,
};

// Unified handler for binary array ops: add, subtract, multiply, divide, bitwise ops
static int mat_math_op(RXIFRM *frm, void *ctx, int op) {
	Mat *src1 = ARG_Mat(1);
	Mat *src2 = ARG_Mat(2);
	Mat *dst  = ARG_Mat_As(3, src1);
	Mat *mask = NULL;
	bool newHandle = FALSE;
	double scale = 1;

	if (op < 10) {
		mask = ARG_Mat(5);
		if(!mask)
			mask = (Mat*)&noArray();
	} else {
		if (!ARG_Is_None(4)) scale = ARG_Double(5);
	}

	if (!src1 || !src2 || !dst ) return RXR_NONE;

	EXCEPTION_TRY
	switch (op){
		case BITWISE_AND:   bitwise_and(*src1, *src2, *dst, *mask); break;
		case BITWISE_OR:    bitwise_or(*src1, *src2, *dst, *mask); break;
		case BITWISE_XOR:   bitwise_xor(*src1, *src2, *dst, *mask); break;
		case MATH_ADD:      add(*src1, *src2, *dst, *mask); break;
		case MATH_SUBTRACT: subtract(*src1, *src2, *dst, *mask); break;
		case MATH_MULTIPLY: multiply(*src1, *src2, *dst, scale); break;
		case MATH_DIVIDE:   divide(*src1, *src2, *dst, scale); break;
	}
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 3);
	return RXR_VALUE;
}


COMMAND cmd_add(RXIFRM *frm, void *ctx) {
	return mat_math_op(frm, ctx, MATH_ADD);
}
COMMAND cmd_subtract(RXIFRM *frm, void *ctx) {
	return mat_math_op(frm, ctx, MATH_SUBTRACT);
}
COMMAND cmd_multiply(RXIFRM *frm, void *ctx) {
	return mat_math_op(frm, ctx, MATH_MULTIPLY);
}
COMMAND cmd_divide(RXIFRM *frm, void *ctx) {
	return mat_math_op(frm, ctx, MATH_DIVIDE);
}
COMMAND cmd_bitwise_and(RXIFRM *frm, void *ctx) {
	return mat_math_op(frm, ctx, BITWISE_AND);
}
COMMAND cmd_bitwise_or(RXIFRM *frm, void *ctx) {
	return mat_math_op(frm, ctx, BITWISE_OR);
}
COMMAND cmd_bitwise_xor(RXIFRM *frm, void *ctx) {
	return mat_math_op(frm, ctx, BITWISE_XOR);
}

COMMAND cmd_bitwise_not(RXIFRM *frm, void *ctx) {
	Mat *src  = ARG_Mat(1);
	Mat *dst  = ARG_Mat_As(2, src);
	Mat *mask = ARG_Mat(3);

	if (!src || !dst ) return RXR_NONE;

	EXCEPTION_TRY
	if (mask) bitwise_not(*src, *dst, *mask);
	else      bitwise_not(*src, *dst);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}

COMMAND cmd_absdiff(RXIFRM *frm, void *ctx) {
	Mat *src1 = ARG_Mat(1);
	Mat *dst  = ARG_Mat_As(3, src1);

	if (!src1 || !dst) return RXR_NONE;

	EXCEPTION_TRY
	if (RXA_TYPE(frm, 2) == RXT_TUPLE) {
		// Accept a Rebol tuple! (e.g. 255.0.0) as a Scalar
		RXIARG arg = RXA_ARG(frm, 2);
		Scalar scalar(arg.bytes[3], arg.bytes[2], arg.bytes[1], arg.bytes[4]);
		absdiff(*src1, scalar, *dst);
	} else {
		Mat *src2 = ARG_Mat(2);
		if (!src2) return RXR_NONE;
		absdiff(*src1, *src2, *dst);
	}
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 6);
	return RXR_VALUE;
}

COMMAND cmd_addWeighted(RXIFRM *frm, void *ctx) {
	Mat *src1    = ARG_Mat(1);
	double alpha = ARG_Double(2);
	Mat *src2    = ARG_Mat(3);
	double beta  = ARG_Double(4);
	double delta = ARG_Double(5);
	Mat *dst     = ARG_Mat_As(6, src1);

	if(!src1 || !src2 || !dst) return RXR_NONE;

	EXCEPTION_TRY
	addWeighted(*src1, alpha, *src2, beta, delta, *dst);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 6);
	return RXR_VALUE;
}

COMMAND cmd_convertScaleAbs(RXIFRM *frm, void *ctx) {
	Mat *src     = ARG_Mat   (1);
	Mat *dst     = ARG_Mat_As(2, src);
	double alpha = ARG_Double(3);
	double beta  = ARG_Double(4);

	if (!src || !dst) return RXR_NONE;

	EXCEPTION_TRY
	convertScaleAbs(*src, *dst, alpha, beta);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}

COMMAND cmd_flip(RXIFRM *frm, void *ctx) {
	Mat *src     = ARG_Mat   (1);
	Mat *dst     = ARG_Mat_As(2, src);
	int flipCode = ARG_Int(3);

	if (!src || !dst) return RXR_NONE;

	EXCEPTION_TRY
	flip(*src, *dst, flipCode);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}

COMMAND cmd_invert(RXIFRM *frm, void *ctx) {
	Mat *src     = ARG_Mat   (1);
	Mat *dst     = ARG_Mat_As(2, src);
	int flipCode = ARG_Int(3);

	if (!src || !dst) return RXR_NONE;

	EXCEPTION_TRY
	invert(*src, *dst, flipCode);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}

COMMAND cmd_max(RXIFRM *frm, void *ctx) {
	Mat *src1 = ARG_Mat(1);
	Mat *src2 = ARG_Mat(2);
	Mat *dst  = ARG_Mat_As(3, src1);

	if (!src1 || !src2) return RXR_NONE;

	EXCEPTION_TRY
	max(*src1, *src2, *dst);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 3);
	return RXR_VALUE;
}

COMMAND cmd_mean(RXIFRM *frm, void *ctx) {
	Scalar scalar;
	REBINT channels;
	RXIARG val;
	REBSER *ser;
	Mat *src = ARG_Mat(1);

	if (!src) return RXR_NONE;

	EXCEPTION_TRY
	scalar = mean(*src);
	EXCEPTION_CATCH

	channels = src->channels();
	if (channels == 1) {
		RXA_TYPE(frm, 1) = RXT_DECIMAL;
		RXA_DEC64(frm, 1) = scalar[0];
	} else {
		ser = (REBSER*)RL_MAKE_BLOCK(channels);
		if(!ser) return RXR_NONE;

		for(int i=0; i < channels; i++) {
			val.dec64 = scalar[i];
			RL_SET_VALUE(ser, i, val, RXT_DECIMAL);
		}
		RXA_TYPE(frm, 1) = RXT_BLOCK;
		RXA_SERIES(frm, 1) = ser;
		RXA_INDEX(frm, 1) = 0;
		SERIES_TAIL(ser) = channels;
	}
	return RXR_VALUE;
}

COMMAND cmd_min(RXIFRM *frm, void *ctx) {
	Mat *src1 = ARG_Mat(1);
	Mat *src2 = ARG_Mat(2);
	Mat *dst  = ARG_Mat_As(3, src1);

	if (!src1 || !src2) return RXR_NONE;

	EXCEPTION_TRY
	min(*src1, *src2, *dst);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 3);
	return RXR_VALUE;
}

COMMAND cmd_minMaxLoc(RXIFRM *frm, void *ctx) {
	Mat *src = ARG_Mat(1);
	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;

	if (!src) return RXR_NONE;

	EXCEPTION_TRY
	minMaxLoc(*src, &minVal, &maxVal, &minLoc, &maxLoc);
	EXCEPTION_CATCH

	REBSER *blk = (REBSER*)RL_MAKE_BLOCK(4);
	RXIARG val;

	val.dec64 = minVal;
	RL_SET_VALUE(blk, 0, val, RXT_DECIMAL);
	val.dec64 = maxVal;
	RL_SET_VALUE(blk, 1, val, RXT_DECIMAL);
	val.pair.x = (float)minLoc.x;
	val.pair.y = (float)minLoc.y;
	RL_SET_VALUE(blk, 2, val, RXT_PAIR);
	val.pair.x = (float)maxLoc.x;
	val.pair.y = (float)maxLoc.y;
	RL_SET_VALUE(blk, 3, val, RXT_PAIR);

	RXA_TYPE(frm, 1) = RXT_BLOCK;
	RXA_SERIES(frm, 1) = blk;
	RXA_INDEX(frm, 1) = 0;
	return RXR_VALUE;
}

COMMAND cmd_normalize(RXIFRM *frm, void *ctx) {
	Mat *src          = ARG_Mat(1);
	Mat *dst          = ARG_Mat_As(2, src);
	double alpha      = ARG_Double(3);
	double beta       = ARG_Double(4);
	int norm_type     = ARG_Int(5);
	Mat *mask         = ARG_Mat(7);

	if(!src || !dst) return RXR_FALSE;

	EXCEPTION_TRY
	if (mask)
		normalize(*src, *dst, alpha, beta, norm_type, dst->type(), *mask);
	else
		normalize(*src, *dst, alpha, beta, norm_type, dst->type());
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}

COMMAND cmd_transform(RXIFRM *frm, void *ctx) {
	Mat *src     = ARG_Mat   (1);
	Mat *dst     = ARG_Mat_As(2, src);
	Mat *m       = ARG_Mat   (3);

	if (!src || !dst || !m) return RXR_NONE;

	EXCEPTION_TRY
	transform(*src, *dst, *m);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}


//;-----------------------------------------------------------------------
//;- Mat class                                                            
//;- https://docs.opencv.org/4.6.0/d3/d63/classcv_1_1Mat.html             
//;-----------------------------------------------------------------------

COMMAND cmd_convertTo(RXIFRM *frm, void *ctx) {
	Mat *src     = ARG_Mat   (1);
	Mat *dst     = ARG_Mat_As(2, src);
	int    rtype = ARG_MatType(3);
	double alpha = ARG_Double(4);
	double beta  = ARG_Double(5);

	if (!src || !dst) return RXR_NONE;

	EXCEPTION_TRY
	src->convertTo(*dst, rtype, alpha, beta);
	EXCEPTION_CATCH

	RXA_ARG(frm, 1) = RXA_ARG(frm, 2);
	return RXR_VALUE;
}
