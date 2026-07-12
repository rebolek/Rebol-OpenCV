//
// Rebol/OpenCV extension — Matrix constructor, helpers, and handle lifecycle
// ====================================
// Defines: cmd_Matrix, all helper functions shared across modules,
// and handle free/get_path callbacks for cvMat, VideoCapture, VideoWriter,
// Trackbar, and MouseCallback.
//

#include "opencv-commands-common.h"

// Global error buffer for exception messages
char* err_buff[255];

// Element byte-size lookup: indexed by OpenCV type (depth | (channels-1)<<3)
unsigned char elementSizeByType[32] = {
	1, 1, 2, 2,  4,  4,  8, 2, //CV_8UC1, CV_8SC1, CV_16UC1, CV_16SC1, CV_32SC1, CV_32FC1, CV_64FC1, CV_16FC1
	2, 2, 4, 4,  8,  8, 16, 4, //CV_8UC2, CV_8SC2, CV_16UC2, CV_16SC2, CV_32SC2, CV_32FC2, CV_64FC2, CV_16FC2
	3, 3, 6, 6, 12, 12, 24, 6, //CV_8UC3, CV_8SC3, CV_16UC3, CV_16SC3, CV_32SC3, CV_32FC3, CV_64FC3, CV_16FC3
	4, 4, 8, 8, 16, 16, 32, 8, //CV_8UC4, CV_8SC4, CV_16UC4, CV_16SC4, CV_32SC4, CV_32FC4, CV_64FC4, CV_16FC4
};

// Rebol vector type enum values → OpenCV depth (CV_8U, CV_8S, …)
int vecType2cvType[12] = {
	CV_8S,  //	VTSI08
	CV_16S, //	VTSI16
	CV_32S, //	VTSI32
	-1,     //	VTSI64
	CV_8U,  //	VTUI08
	CV_16U, //	VTUI16
	-1,     //	VTUI32
	-1,     //	VTUI64
	-1,     //	VTSF08
	-1,     //	VTSF16
	CV_32F, //	VTSF32
	CV_64F  //	VTSF64
};


extern "C" {

//----------------------------------------------------------------------
// initRXHandleArg — store a handle + series into an RXIARG value
//----------------------------------------------------------------------
int initRXHandleArg(RXIARG* val, void* handle, REBCNT type, REBSER* ser) {
	REBHOB* hob = RL_MAKE_HANDLE_CONTEXT(type);
	debug_print("new hob: %p handle: %p\n", hob, handle);
	if (hob == NULL) return RXR_FALSE;
	hob->handle = handle;
	hob->series = ser;
	val->handle.ptr = hob;
	val->handle.type = type;
	val->handle.flags = hob->flags;
	return RXR_VALUE;
}

//----------------------------------------------------------------------
// initRXHandle — set frame arg[index] to a new handle value
//----------------------------------------------------------------------
int initRXHandle(RXIFRM *frm, int index, void* handle, REBCNT type, REBSER* ser) {
	RXA_TYPE(frm, index) = RXT_HANDLE;
	return initRXHandleArg(&RXA_ARG(frm, index), handle, type, ser);
}

//----------------------------------------------------------------------
// new_Mat_From_Image_Arg — wrap a Rebol image!'s pixel data in a cv::Mat
//----------------------------------------------------------------------
Mat* new_Mat_From_Image_Arg(RXIFRM *frm, int index) {
	RXIARG arg = RXA_ARG(frm, index);
	Mat *mat = new Mat(arg.height, arg.width, CV_8UC4);
	mat->data = ((REBSER*)arg.series)->data;
	return mat;
}

//----------------------------------------------------------------------
// new_Reb_Vector — allocate a Rebol vector! matching the OpenCV depth
//----------------------------------------------------------------------
REBSER* new_Reb_Vector(REBCNT size, int depth) {
	REBINT type; // int = 0, decimal = 1
	REBINT sign; // 0 = signed, 1 = unsigned
	REBINT bits, dims = 1;
	switch(depth) {
		case CV_8U:  type = 0; sign = 1; bits = 8;  break;
		case CV_8S:  type = 0; sign = 0; bits = 8;  break;
		case CV_16U: type = 0; sign = 1; bits = 16; break;
		case CV_16S: type = 0; sign = 0; bits = 16; break;
		case CV_32S: type = 0; sign = 0; bits = 32; break;
		case CV_32F: type = 1; sign = 0; bits = 32; break;
		case CV_64F: type = 1; sign = 0; bits = 64; break;
	}
	REBSER *vec = (REBSER *)RL_MAKE_VECTOR(type, sign, dims, bits, size);
	SERIES_TAIL(vec) = size;
	return vec;
}

//----------------------------------------------------------------------
// mat2ser — copy cv::Mat pixel data into a Rebol binary series
// Handles non-contiguous matrices by iterating rows.
//----------------------------------------------------------------------
void mat2ser(Mat *mat, REBSER *bin, RXIARG *arg) {
	size_t elemSize = mat->elemSize();
	size_t bytes = elemSize * mat->cols * mat->rows;

	if (mat->isContinuous()) {
		memcpy(bin->data, mat->data,  bytes);
	} else {
		unsigned char *bp = mat->data;
		unsigned char *dp = bin->data;
		size_t wb = elemSize * mat->cols;
		size_t st = mat->step;
		for( int j = 0; j < mat->rows; j++ ) {
			memcpy(dp, bp, wb);
			bp += st;
			dp += wb;
		}
	}
	SERIES_TAIL(bin) = bytes;
	arg->series = bin;
	arg->index = 0;
}

//----------------------------------------------------------------------
// Common_mold — format a handle for Rebol's mold (display) action
//----------------------------------------------------------------------
int Common_mold(REBHOB *hob, REBSER *str) {
	int len;
	if (!str) return 0;
	SERIES_TAIL(str) = 0;
	APPEND_STRING(str, "0#%lx", (unsigned long)(uintptr_t)hob->data);
	return len;
}

//----------------------------------------------------------------------
// cvMat_free / cvMat_get_path — lifecycle for cvMat handles
//----------------------------------------------------------------------
void* cvMat_free(void* cls) {
	debug_print("GC Mat class %p\n", cls);
	if (cls != NULL) {
		Mat *mat = (Mat*)cls;
		mat->release();
	}
	return NULL;
}

int cvMat_get_path(REBHOB *hob, REBCNT word, REBCNT *type, RXIARG *arg) {
	Size size;
	Mat tmp;
	Mat *mat = (Mat*)hob->handle;
	if (!mat) return RXR_NONE;
	REBSER* ser = hob->series;
	REBSER* bin;
	int channels;

	word = RL_FIND_WORD(arg_words, word);

	switch(word){
	case W_ARG_SIZE:
		*type = RXT_PAIR;
		size = mat->size();
		arg->dec32a = size.width;
		arg->dec32b = size.height;
		break;
	case W_ARG_WIDTH:
	case W_ARG_HEIGHT:
		*type = RXT_INTEGER;
		size = mat->size();
		arg->uint64 = (word == W_ARG_WIDTH) ? size.width : size.height;
		break;
	case W_ARG_TYPE:
		*type = RXT_WORD;
		arg->int64 = type_words[mat->type() + W_TYPE_CV_8UC1];
		break;
	case W_ARG_DEPTH:
		*type = RXT_WORD;
		arg->int64 = type_words[mat->depth() + W_TYPE_CV_8U];
		break;
	case W_ARG_CHANNELS:
		*type = RXT_INTEGER;
		arg->uint64 = mat->channels();
		break;
	case W_ARG_BINARY:
		*type = RXT_BINARY;
		if (ser) {
			arg->series = ser;
			arg->index = 0;
		} else {
			bin = (REBSER *)RL_MAKE_STRING(mat->elemSize() * mat->cols * mat->rows, FALSE);
			mat2ser(mat, bin, arg);
		}
		break;
	case W_ARG_IMAGE:
		*type = RXT_IMAGE;
		channels = mat->channels();
		if (channels == 1) {
			cvtColor(*mat, tmp, COLOR_GRAY2BGRA);
		}
		else if (channels == 3 || channels == 4) {
			cvtColor(*mat, tmp, COLOR_BGR2BGRA);
		}
		else {
			*type = RXT_NONE;
			break;
		}
		bin = (REBSER *)RL_MAKE_IMAGE(tmp.cols, tmp.rows);
		mat2ser(&tmp, bin, arg);
		SERIES_TAIL(bin) = tmp.cols * tmp.rows;
		arg->width  = tmp.cols;
		arg->height = tmp.rows;
		break;
	case W_ARG_VECTOR:
		*type = RXT_VECTOR;
	{
		// Copy pixel data into a new typed Rebol vector.
		// The matrix may share a binary! buffer (ser), but binary! cannot be
		// reinterpreted as vector! through the current extension API, so we
		// always allocate a fresh vector and copy.
		REBSER *vec = new_Reb_Vector(mat->cols * mat->rows * mat->channels(), mat->depth());
		mat2ser(mat, vec, arg);
	}
		break;
	case W_ARG_TOTAL:
		*type = RXT_INTEGER;
		arg->uint64 = mat->total();
		break;
	case W_ARG_IS_SUBMATRIX:
		*type = RXT_LOGIC;
		arg->int32a = mat->isSubmatrix();
		break;
	default:
		return PE_BAD_SELECT;
	}
	return PE_USE;
}

//----------------------------------------------------------------------
// cvVideoCapture_free / cvVideoCapture_get_path — lifecycle for VideoCapture
//----------------------------------------------------------------------
void* cvVideoCapture_free(void* cls) {
	debug_print("GC VideoCapture class %p\n", cls);
	if (cls != NULL) {
		VideoCapture *cap = (VideoCapture*)cls;
		cap->release();
	}
	return NULL;
}

int cvVideoCapture_get_path(REBHOB *hob, REBCNT word, REBCNT *type, RXIARG *arg) {
	VideoCapture *cap= (VideoCapture*)hob->handle;
	if (!cap) return RXR_NONE;

	word = RL_FIND_WORD(arg_words, word);

	if (word == W_ARG_WIDTH) {
		*type = RXT_INTEGER;
		arg->uint64 = cap->get(CAP_PROP_FRAME_WIDTH);
		return PE_USE;
	}
	if (word == W_ARG_HEIGHT) {
		*type = RXT_INTEGER;
		arg->uint64 = cap->get(CAP_PROP_FRAME_HEIGHT);
		return PE_USE;
	}
	if (word == W_ARG_FRAMES) {
		*type = RXT_INTEGER;
		arg->uint64 = cap->get(CAP_PROP_FRAME_COUNT);
		return PE_USE;
	}
	if (word == W_ARG_FORMAT) {
		*type = RXT_WORD;
		arg->int64 = type_words[(int)cap->get(CAP_PROP_FORMAT) + W_TYPE_CV_8UC1];
		return PE_USE;
	}
	if (word >= W_ARG_POS_MS && word <= W_ARG_FRAMES) {
		*type = RXT_DECIMAL;
		arg->dec64 = cap->get(word - W_ARG_POS_MS);
		return PE_USE;
	}
	return PE_BAD_SELECT;
}

//----------------------------------------------------------------------
// cvMouseCallback_free / cvMouseCallback_get_path — lifecycle for mouse callbacks
//----------------------------------------------------------------------
void* cvMouseCallback_free(void* cls) {
	debug_print("GC MouseCallback class %p\n", cls);
	if (cls != NULL) {
		CTX_MOUSECALLBACK *mcb = (CTX_MOUSECALLBACK*)cls;
		if (mcb->window != NULL) {
			delete mcb->window;
		}
		if(mcb->cbi)  FREE_MEM(mcb->cbi);
		if(mcb->args) FREE_MEM(mcb->args);
	}
	return NULL;
}

int cvMouseCallback_get_path(REBHOB *hob, REBCNT word, REBCNT *type, RXIARG *arg) {
	CTX_MOUSECALLBACK *mcb = (CTX_MOUSECALLBACK*)hob->handle;
	if (!mcb) return RXR_NONE;

	word = RL_FIND_WORD(arg_words, word);
	if (word == W_ARG_X) {
		*type = RXT_INTEGER;
		arg->int64 = mcb->x;
		return PE_USE;
	}
	if (word == W_ARG_Y) {
		*type = RXT_INTEGER;
		arg->int64 = mcb->y;
		return PE_USE;
	}
	if (word == W_ARG_POS) {
		*type = RXT_PAIR;
		arg->dec32a = (float)mcb->x;
		arg->dec32b = (float)mcb->y;
		return PE_USE;
	}
	if (word == W_ARG_FLAGS) {
		*type = RXT_INTEGER;
		arg->int64 = mcb->flags;
		return PE_USE;
	}
	return PE_BAD_SELECT;
}


//----------------------------------------------------------------------
// releaseVideoWriter — GC callback for VideoWriter handles
//----------------------------------------------------------------------
void* releaseVideoWriter(void* cls) {
	debug_print("GC VideoWriter class %p\n", cls);
	if (cls != NULL) {
		VideoWriter *cap = (VideoWriter*)cls;
		cap->release();
	}
	return NULL;
}

//----------------------------------------------------------------------
// releaseTrackbar — GC callback for Trackbar handles
//----------------------------------------------------------------------
void* releaseTrackbar(void* cls) {
	debug_print("GC cvTrackbar %p\n", cls);
	if (cls != NULL) {
		CTX_TRACKBAR *bar = (CTX_TRACKBAR*)cls;
		delete bar->name;
		delete bar->window;
		if(bar->cbi)  FREE_MEM(bar->cbi);
		if(bar->args) FREE_MEM(bar->args);
	}
	return NULL;
}

} // extern "C"


//;-----------------------------------------------------------------------
//;- Matrix constructor                                                   
//;-----------------------------------------------------------------------

COMMAND cmd_Matrix(RXIFRM *frm, void *ctx) {
	Mat *mat = NULL;
	REBSER *bin = NULL;
	REBSER *blk;
	REBCNT n, t;
	REBOOL resolved;
	REBOOL sharedBin = FALSE;
	RXIARG val;
	Size size = Size(0,0);
	int type = CV_8UC4;
	int binBytes = 0, matBytes = 0;

	if (ARG_Is_Block(1)) {
		// Block spec: [size [type] data ...]
		// Accepted items: pair (size), word (type), tuple (fill color),
		// binary (pixel data), vector (pixel data), cvMat handle + [rect]
		blk = (REBSER *)RXA_SERIES(frm, 1);

		for(n = RXA_INDEX(frm, 1); (t = RL_GET_VALUE(blk, n, &val)); n++) {
			if (t == RXT_END) break;
			if (t == RXT_GET_WORD || t == RXT_GET_PATH) {
				resolved = TRUE;
				t = RL_GET_VALUE_RESOLVED(blk, n, &val);
			} else {
				resolved = FALSE;
			}
			if (t == RXT_PAIR) {
				size = Size((int)val.pair.x, (int)val.pair.y);
			}
			else if (t == RXT_WORD) {
				type = RL_FIND_WORD(type_words, val.int32a);
				if (type < W_TYPE_CV_8UC1 || type > W_TYPE_CV_16FC4) return RXR_FALSE;
				type -= W_TYPE_CV_8UC1;
			}
			else if (t == RXT_TUPLE) {
				// Fill with a color (tuple)
				if (RXT_END != RL_GET_VALUE(blk, ++n, &val)) goto err_spec;
				if (size.width <= 0 || size.height <= 0) goto err_size;
				mat = new Mat(size, type, Scalar(val.bytes[3],val.bytes[2],val.bytes[1],val.bytes[4]));
				goto done;
			}
			else if (t == RXT_BINARY) {
				bin = (REBSER*)val.series;
				binBytes = SERIES_TAIL(bin) - val.index;
				sharedBin = resolved;
			}
			else if (t == RXT_VECTOR) {
				bin = (REBSER*)val.series;
				if (size.width <= 0 || size.height <= 0) goto err_size;
				int vecType = VECT_TYPE(bin);
				if (vecType < 0 || vecType > 11 || (type = vecType2cvType[vecType]) < 0 ) {
					trace("Invalid vector type.");
					return RXR_FALSE;
				}
				int depth = (bin->tail - val.index) / (size.width * size.height);
				binBytes = (SERIES_TAIL(bin) - val.index) * VECT_BYTE_SIZE(vecType);
				type = CV_MAKETYPE(type, depth);
				if (type < 0 || type > CV_16FC4) goto err_vect;
				sharedBin = resolved;
			}
			else if (t == RXT_HANDLE) {
				// Clone from a cvMat, optionally with a ROI rect
				Mat *src;
				REBSER *rect;
				RXIARG pos, sz;

				if (val.handle.type != Handle_cvMat) {
					trace("Expected cvMat handle.");
					goto err_spec;
				}
				src = (Mat*)val.handle.hob->data;
				if (!src) {
					trace("Invalid matrix argument.");
					goto err_spec;
				}
				t = RL_GET_VALUE(blk, n++, &val);
				if (t == RXT_GET_WORD || t == RXT_GET_PATH) {
					t = RL_GET_VALUE_RESOLVED(blk, n, &val);
				}
				if (t != RXT_BLOCK) {
					trace("Expected block!");
					goto err_spec;
				}
				rect = (REBSER*)val.series;
				if (RXT_PAIR != RL_GET_VALUE(rect, val.index, &pos) || RXT_PAIR != RL_GET_VALUE(rect, val.index+1, &sz)) {
					trace("Expected block with 2 pairs!");
					goto err_spec;
				}
				mat = new Mat(*src, Rect(pos.pair.x,pos.pair.y,sz.pair.x,sz.pair.y));
				goto done;
			}
			else goto err_spec;
		}
		if (bin) {
			// Construct from binary or vector data
			if (size.width <= 0 || size.height <= 0) goto err_size;
			if (type < 0 || type > CV_16FC4) goto err_type;

			matBytes =  size.width * size.height * elementSizeByType[type];

			if (sharedBin) {
				if(binBytes < matBytes) goto err_size;
				mat = new Mat(size, type, SERIES_SKIP(bin, val.index));
			}
			else {
				REBSER *ser = NULL;
				if (matBytes > 0) {
					ser = (REBSER *)RL_MAKE_STRING(matBytes, FALSE);
					SERIES_TAIL(ser) = matBytes;
					mat = new Mat(size, type, ser->data);

					if (binBytes < matBytes) {
						unsigned char *bp = ser->data;
						for (;binBytes <= matBytes; bp += binBytes, matBytes--) {
							memcpy(bp, SERIES_SKIP(bin, val.index), binBytes);
						}
					} else {
						memcpy(ser->data, SERIES_SKIP(bin, val.index), matBytes);
					}
				}
				*bin = *ser;
			}
			goto done;
		}
	}
	else if (ARG_Is_Image(1)) {
		mat = new_Mat_From_Image_Arg(frm, 1);
		goto done;
	}
	else if (ARG_Is_Mat(1)) {
		mat = ARG_Mat(1);
		if (!mat) return RXR_NONE;
		size = mat->size();
		type = mat->type();
	}
	else if (ARG_Is_Pair(1)) {
		size = ARG_Size(1);
		if (size.width < 0 || size.height < 0) goto err_size;
		mat = new Mat(size, type, Scalar(0,0,0));
		goto done;
	}
	mat = new Mat(size, type);

done:
	return initRXHandle(frm, 1, mat, Handle_cvMat, bin);
err_size:
	trace("Invalid or missing size specification!");
	return RXR_FALSE;
err_spec:
	debug_print("Invalid matrix spec at index... %u\n", n);
	return RXR_FALSE;
err_shortBin:
	debug_print("Insufficient series size! Requited %i, but got only %i bytes.", matBytes, binBytes);
	return RXR_FALSE;
err_type:
	debug_print("Invalid matrix type %i!\n", type);
	return RXR_FALSE;
err_vect:
	trace("Vector size does not match with the given matrix size!");
	return RXR_FALSE;
}
