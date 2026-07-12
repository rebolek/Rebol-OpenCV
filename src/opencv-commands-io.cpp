//
// Rebol/OpenCV extension — Image I/O and Video I/O
// ====================================
// Defines: imread, imreadmulti, imwrite, VideoCapture, VideoWriter,
// read, write, free, get-property, set-property.
//

#include "opencv-commands-common.h"


//;-----------------------------------------------------------------------
//;- Handle free                                                          
//;-----------------------------------------------------------------------

COMMAND cmd_free(RXIFRM *frm, void *ctx) {
	REBCNT type = RXA_HANDLE_TYPE(frm, 1);
	if (type) {
		RL_FREE_HANDLE_CONTEXT((REBHOB*)RXA_HANDLE_CONTEXT(frm, 1));
	}
	else {
		return RXR_NONE;
	}
	return RXR_TRUE;
}


//;-----------------------------------------------------------------------
//;- Property getter/setter                                               
//;-----------------------------------------------------------------------

COMMAND cmd_get_property(RXIFRM *frm, void *ctx) {
	int propid = RXA_INT32(frm, 2);
	double result = 0;

	if (!RXA_HANDLE_CONTEXT(frm, 1)) return RXR_FALSE;

	if (ARG_Is_VideoCapture(1)) {
		VideoCapture *cap = ARG_VideoCapture(1);
		result = cap->get(propid);
	}
	else if (ARG_Is_Mat(1)) {
		REBHOB* hob = RXA_HANDLE_CONTEXT(frm, 1);
		if (!hob) return RXR_NONE;
		Mat *mat = (Mat*)hob->handle;
		if (!mat) return RXR_NONE;
		REBSER* ser = hob->series;

		switch(propid){
			case MAT_SIZE: {
				Size size = mat->size();
				RXA_TYPE(frm,1) = RXT_PAIR;
				RXA_PAIR(frm,1).x = size.width;
				RXA_PAIR(frm,1).y = size.height;
				return RXR_VALUE;
			}
			case MAT_TYPE: {
				RXA_TYPE(frm,1) = RXT_WORD;
				RXA_ARG(frm,1).int64 = type_words[mat->type() + W_TYPE_CV_8UC1];
				return RXR_VALUE;
			}
			case MAT_DEPTH: {
				RXA_TYPE(frm,1) = RXT_INTEGER;
				RXA_ARG(frm,1).int64 = mat->depth();
				return RXR_VALUE;
			}
			case MAT_CHANNELS: {
				RXA_TYPE(frm,1) = RXT_INTEGER;
				RXA_ARG(frm,1).int64 = mat->channels();
				return RXR_VALUE;
			}
			case MAT_BINARY: {
				if (ser) {
					RXA_SERIES(frm, 1) = ser;
					RXA_INDEX(frm, 1) = 0;
				} else {
					REBSER *bin = (REBSER *)RL_MAKE_STRING(mat->elemSize() * mat->cols * mat->rows, FALSE);
					mat2ser(mat, bin, &RXA_ARG(frm,1));
				}
				RXA_TYPE  (frm, 1) = RXT_BINARY;
				return RXR_VALUE;
			}
			case MAT_IMAGE: {
				Mat tmp;
				int channels = mat->channels();
				if (channels == 1)
					cvtColor(*mat, tmp, COLOR_GRAY2BGRA);
				else if (channels == 3 || channels == 4)
					cvtColor(*mat, tmp, COLOR_BGR2BGRA);
				else return RXR_NONE;

				REBSER *bin = (REBSER *)RL_MAKE_IMAGE(tmp.cols, tmp.rows);
				mat2ser(&tmp, bin, &RXA_ARG(frm,1));
				RXA_TYPE(frm, 1) = RXT_IMAGE;
				RXA_ARG(frm, 1).width  = tmp.cols;
				RXA_ARG(frm, 1).height = tmp.rows;
				return RXR_VALUE;
			}
			case MAT_VECTOR: {
				if (ser) {
					RXA_SERIES(frm, 1) = ser;
					RXA_INDEX(frm, 1) = 0;
					//TODO: propper casting from binary series to vector of type used by the Matrix
					//     (which needs modification on Rebol side)
				} else {
					REBSER *vec = new_Reb_Vector(mat->cols * mat->rows * mat->channels(), mat->depth());
					mat2ser(mat, vec, &RXA_ARG(frm,1));
				}
				RXA_TYPE  (frm, 1) = RXT_VECTOR;
				return RXR_VALUE;
			}
		}
	}
	else {
		return RXR_NONE;
	}
	RXA_TYPE(frm, 1) = RXT_DECIMAL;
	RXA_DEC64(frm, 1) = result;
	return RXR_VALUE;
}

COMMAND cmd_set_property(RXIFRM *frm, void *ctx) {
	int propid   = RXA_INT32(frm, 2);
	double value = RXA_DEC64(frm, 3);
	bool result = FALSE;

	if (!RXA_HANDLE_CONTEXT(frm, 1)) return RXR_FALSE;

	EXCEPTION_TRY
	if (ARG_Is_VideoCapture(1)) {
		VideoCapture *cap = ARG_VideoCapture(1);
		result = cap->set(propid, value);
	}
	EXCEPTION_CATCH

	RXA_TYPE(frm, 1) = RXT_LOGIC;
	RXA_LOGIC(frm, 1) = result;
	return RXR_VALUE;
}


//;-----------------------------------------------------------------------
//;- VideoCapture / VideoWriter constructors and read/write               
//;-----------------------------------------------------------------------

COMMAND cmd_VideoCapture(RXIFRM *frm, void *ctx) {
	VideoCapture *cap;
	int deviceID;

	EXCEPTION_TRY

	cap = new VideoCapture();

	if (RXA_TYPE(frm, 1) == RXT_INTEGER) {
		cap->open(ARG_Int(1), CAP_ANY);
	} else {
		cap->open(ARG_String(1), CAP_ANY);
	}

	EXCEPTION_CATCH

	debug_print("cap %p\n", cap);
	if (!cap->isOpened()) {
		cap->release();
		return RXR_FALSE;
	}

	return initRXHandle(frm, 1, cap, Handle_cvVideoCapture, NULL);
}

COMMAND cmd_VideoWriter(RXIFRM *frm, void *ctx) {
	VideoWriter *writer;

	String name  = ARG_String(1);
	int    codec = ARG_Int(2);
	double fps   = ARG_Double(3);
	Size   size  = ARG_Size(4);

	EXCEPTION_TRY

	if (!codec) codec = VideoWriter::fourcc('M', 'J', 'P', 'G');

	writer = new VideoWriter();

	writer->open(name, codec, fps, size, true);
	if (!writer->isOpened()) {
		puts("Not opened!");
		writer->release();
		return RXR_FALSE;
	}

	EXCEPTION_CATCH

	return initRXHandle(frm, 1, writer, Handle_cvVideoWriter, NULL);
}

COMMAND cmd_read(RXIFRM *frm, void *ctx) {
	VideoCapture *cap;
	Mat *frame;

	if(!ARG_Is_VideoCapture(1))
		return RXR_FALSE;

	cap = ARG_VideoCapture(1);
	if (!cap->isOpened()) return RXR_NONE;

	frame = ARG_Mat(3);
	if (!frame) {
		frame = new Mat();
		if (RXR_VALUE != initRXHandle(frm, 1, frame, Handle_cvMat, NULL))
			return RXR_FALSE;
	}

	EXCEPTION_TRY
	cap->read(*frame);
	EXCEPTION_CATCH

	if (frame->empty()) return RXR_NONE;

	return RXR_VALUE;
}

COMMAND cmd_write(RXIFRM *frm, void *ctx) {
	VideoWriter *writer;

	if(!ARG_Is_VideoWriter(1))
		return RXR_FALSE;

	writer = ARG_VideoWriter(1);
	if (!writer || !writer->isOpened()) return RXR_NONE;

	EXCEPTION_TRY
	if (ARG_Is_Mat(2)) {
		Mat *mat = ARG_Mat(2);
		if (!mat) return RXR_FALSE;
		writer->write(*mat);
	} else {
		Mat image, mat;
		RXIARG arg = RXA_ARG(frm, 2);
		image = Mat(arg.height, arg.width, CV_8UC4);
		image.data = ((REBSER*)arg.series)->data;
		image.convertTo(mat, CV_8UC3);
		writer->write(mat);
	}
	EXCEPTION_CATCH

	return RXR_TRUE;
}


//;-----------------------------------------------------------------------
//;- Image file reading and writing                                       
//;- https://docs.opencv.org/4.6.0/d4/da8/group__imgcodecs.html           
//;-----------------------------------------------------------------------

COMMAND cmd_imread(RXIFRM *frm, void *ctx) {
	Mat image;
	String filename = ARG_String(1);
	int flags = (ARG_Is_Integer(4) ? RXA_INT32(frm,4) : IMREAD_UNCHANGED);

	EXCEPTION_TRY
	image = imread(filename, flags);
	if (image.empty()) return RXR_NONE;
	if (ARG_Is_None(2)) {
		Mat *result = new Mat(image);
		if (RXR_VALUE != initRXHandle(frm, 1, result, Handle_cvMat, NULL))
			return RXR_FALSE;
	}
	else {
		REBSER* reb_image;
		cvtColor(image, image, COLOR_BGR2BGRA);
		reb_image = (REBSER *)RL_MAKE_IMAGE(image.cols, image.rows);
		memcpy(reb_image->data, image.data, 4*image.cols*image.rows);

		RXA_TYPE(frm, 1) = RXT_IMAGE;
		RXA_ARG(frm, 1).width  = image.cols;
		RXA_ARG(frm, 1).height = image.rows;
		RXA_ARG(frm, 1).image  = reb_image;
	}

	EXCEPTION_CATCH

	return RXR_VALUE;
}

COMMAND cmd_imreadmulti(RXIFRM *frm, void *ctx) {
	vector<Mat> images;
	String filename = ARG_String(1);
	int flags = (ARG_Is_Integer(4) ? RXA_INT32(frm,4) : IMREAD_UNCHANGED);

	EXCEPTION_TRY
	if (!imreadmulti(filename, images, flags)) return RXR_FALSE;
	EXCEPTION_CATCH

	int n;
	int num = images.size();
	REBSER *blk = (REBSER*)RL_MAKE_BLOCK(num);
	RXIARG val;

	if (ARG_Is_None(2)) {
		Mat *image;
		for (n = 0; n < num; n++) {
			image = new Mat(images[n]);
			initRXHandleArg(&val, image, Handle_cvMat, NULL);
			RL_SET_VALUE(blk, n, val, RXT_HANDLE);
		}
	} else {
		Mat image;
		REBSER* reb_image;
		for (n = 0; n < num; n++) {
			cvtColor(images[n], image, COLOR_BGR2BGRA);
			reb_image = (REBSER *)RL_MAKE_IMAGE(image.cols, image.rows);
			memcpy(reb_image->data, image.data, 4*image.cols*image.rows);
			val.width  = image.cols;
			val.height = image.rows;
			val.image  = reb_image;
			RL_SET_VALUE(blk, n, val, RXT_IMAGE);
		}
	}
	RXA_TYPE(frm, 1) = RXT_BLOCK;
	RXA_SERIES(frm, 1) = blk;
	return RXR_VALUE;
}

COMMAND cmd_imwrite(RXIFRM *frm, void *ctx) {
	String name = ARG_String(1);
	bool result = false;
	vector<int> params = vector<int>();

	EXCEPTION_TRY
	if (RXA_TYPE(frm, 4) == RXT_BLOCK) {
		REBSER *cmds;
		REBCNT index, type;
		RXIARG arg1,arg2;
		cmds = (REBSER*)RXA_SERIES(frm, 4);
		index = RXA_INDEX(frm, 4);
		while (index+1 < cmds->tail
			&& RXT_INTEGER == RL_GET_VALUE_RESOLVED(cmds, index++, &arg1)
			&& RXT_INTEGER == RL_GET_VALUE_RESOLVED(cmds, index++, &arg2)) {
			params.push_back(arg1.int32a);
			params.push_back(arg2.int32a);
		}
	}

	if (ARG_Is_Mat(2)) {
		if (!ARG_Mat(2)) return RXR_FALSE;
		result = imwrite(name, *ARG_Mat(2), params);
	} else if(ARG_Is_Image(2)) {
		Mat image;
		RXIARG arg = RXA_ARG(frm, 2);
		image = Mat(arg.height, arg.width, CV_8UC4);
		image.data = ((REBSER*)arg.series)->data;
		result = imwrite(name, image, params);
	}
	EXCEPTION_CATCH

	return result ? RXR_VALUE : RXR_FALSE;
}
