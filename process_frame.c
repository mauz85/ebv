/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

#define IMG_SIZE NUM_COLORS*(OSC_CAM_MAX_IMAGE_WIDTH/2)*(OSC_CAM_MAX_IMAGE_HEIGHT/2)

const int nc = OSC_CAM_MAX_IMAGE_WIDTH/2;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT/2;

int Border = 6;
int helpBuf[IMG_SIZE];
int avgDxy[3][IMG_SIZE];

int TextColor;
void CalcDeriv();
void AvgDeriv(int Index);

int k_val = 5;
void EdgeStrength();

int locMaxDimension = 9;
int maxMc = 0;

void detLocalMaximas();
void detLocalMaximas2();

int SizeBox = 5;
void drawMaxMc();

void ResetProcess()
{
	//called when "reset" button is pressed
	if(TextColor == CYAN)
		TextColor = MAGENTA;
	else
		TextColor = CYAN;
}


void ProcessFrame()
{
	uint32 t1, t2;
	char Text[] = "hallo world";
	//initialize counters
	if(data.ipc.state.nStepCounter == 1) {
		//use for initialization; only done in first step
		memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
		TextColor = CYAN;
	} else {
		//example for time measurement
		t1 = OscSupCycGet();
		//example for copying sensor image to background image
		//memcpy(data.u8TempImage[BACKGROUND], data.u8TempImage[SENSORIMG], IMG_SIZE);
		CalcDeriv();

		AvgDeriv(1);
		AvgDeriv(2);
		AvgDeriv(0);

		EdgeStrength();
		detLocalMaximas2();
		drawMaxMc();

		//example for time measurement
		t2 = OscSupCycGet();

		//example for log output to console
		OscLog(INFO, "required = %d us\n", OscSupCycToMicroSecs(t2-t1));

		//example for drawing output
		//draw line
		// DrawLine(10, 100, 200, 20, RED);
		//draw open rectangle
		//DrawBoundingBox(20, 10, 50, 40, false, GREEN);
		//draw filled rectangle
		// DrawBoundingBox(80, 100, 110, 120, true, BLUE);
		// DrawString(200, 200, strlen(Text), TINY, TextColor, Text);
	}
}

void CalcDeriv()
{
	int c, r;
	for(r = nc; r < nr*nc-nc; r+= nc) {/* we skip the first and last line */
		for(c = 1; c < nc-1; c++) {
			/* do pointer arithmetics with respect to center pixel location */
			unsigned char* p = &data.u8TempImage[SENSORIMG][r+c];
			/* implement Sobel filter */
			int dx = -(int) *(p-nc-1) + (int) *(p-nc+1)
						-2* (int) *(p-1) + 2* (int) *(p+1)
						-(int) *(p+nc-1) + (int) *(p+nc+1);
			int dy = (int) *(p-nc-1) + (int) *(p-nc) * 2 + (int) *(p-nc+1)
					-(int) *(p+nc-1) - (int) *(p+nc) * 2 - (int) *(p+nc+1);

			//not yet averaged!!
			avgDxy[0][r+c] = dx*dx;
			avgDxy[1][r+c] = dy*dy;
			avgDxy[2][r+c] = dx*dy;

			//data.u8TempImage[BACKGROUND][r+c] = (uint8) MIN(255, MAX(0, (dx*dx) >> 10));
			//data.u8TempImage[THRESHOLD][r+c] = (uint8) MIN(255, MAX(0, 128+dy));
		}
	}
}

void AvgDeriv(int Index) {

	//do average in x-direction
	int c, r;
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &avgDxy[Index][r + c];
			int sx = (*(p - 6) + *(p + 6)) * 1 + (*(p - 5) + *(p + 5)) * 4
					+ (*(p - 4) + *(p + 4)) * 11 + (*(p - 3) + *(p + 3)) * 27
					+ (*(p - 2) + *(p + 2)) * 50 + (*(p - 1) + *(p + 1)) * 72
					+ (*p) * 82;
			//now averaged
			helpBuf[r + c] = (sx >> 8);
		}
	}
	//do average in y-direction
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &helpBuf[r + c];
			int sy = (*(p - 6*nc) + *(p + 6*nc)) * 1 + (*(p - 5*nc) + *(p + 5*nc)) * 4
					+ (*(p - 4*nc) + *(p + 4*nc)) * 11 + (*(p - 3*nc) + *(p + 3*nc)) * 27
					+ (*(p - 2*nc) + *(p + 2*nc)) * 50 + (*(p - 1*nc) + *(p + 1*nc)) * 72
					+ (*p) * 82;
			//now averaged
			avgDxy[Index][r+c] = (sy >> 8);
			data.u8TempImage[THRESHOLD][r+c] = (uint8)  MIN(255, MAX(0,(sy >> 12)));
		}
	}
}

void EdgeStrength()
{
	int r, c;
	for(r = nc * (Border+1); r < nr * nc -(Border+1) * nc; r+=nc){
		for(c = Border+1;c < nc - (Border+1); c++) {
			int dlx2 = ((avgDxy[0][r+c]) >> 7);
			int dly2 = ((avgDxy[1][r+c]) >> 7);
			int dlxy = ((avgDxy[2][r+c]) >> 7);

			avgDxy[0][r+c] = (dlx2*dly2-dlxy*dlxy) - ((k_val*((dlx2+dly2)*(dlx2+dly2))) >> 7);
			data.u8TempImage[BACKGROUND][r+c] = (uint8) MIN(255, MAX(0, (avgDxy[0][r+c] >> 10)));
		}
	}
}

void detLocalMaximas()
{
	int r, c, lmr, lmc, maxima, tmp;
	int dim = locMaxDimension / 2;

	for(r = nc * (dim + Border+1); r < nr * nc -(dim + Border+1) * nc; r+=nc){
		for(c = dim + Border+1;c < nc - (dim + Border+1); c++) {

			maxima = 0;
			int* p = &avgDxy[0][r + c];
			for (lmr = -dim; lmr < dim; lmr++) {
				for (lmc = -dim; lmc < dim; lmc++) {
					tmp = *(p + lmr * nc + lmc);
					maxima = (tmp > maxima) ? tmp : maxima;
				}
			}
			helpBuf[r + c] = maxima;
			data.u8TempImage[THRESHOLD][r+c] = (uint8)  (uint8) MIN(255, MAX(0, (maxima >> 10)));
		}
	}
}

void detLocalMaximas2()
{
	//do average in x-direction
	int c, r, lmr, lmc, tmp, maxima;
	int dim = locMaxDimension / 2;
	for(r = nc * (Border+1); r < nr * nc -(Border+1) * nc; r+=nc){
		for(c = dim + Border+1;c < nc - (dim + Border+1); c++) {
			/* do pointer arithmetics with respect to center pixel location */
			maxima = 0;
			int* p = &avgDxy[0][r + c];
			for (lmc = -dim; lmc < dim; lmc++) {
				tmp = *(p + lmc);
				maxima = (tmp > maxima) ? tmp : maxima;
			}
			helpBuf[r + c] = maxima;
		}
	}

	maxMc = 0;
	OscLog(INFO, "maxMc = %d\n", maxMc);
	//do average in y-direction
	for(r = nc * (dim + Border+1); r < nr * nc -(dim + Border+1) * nc; r+=nc){
		for(c = dim + Border+1;c < nc - (dim + Border+1); c++) {
			/* do pointer arithmetics with respect to center pixel location */
			maxima = 0;
			int* p = &helpBuf[r + c];
			for (lmr = -dim; lmr < dim; lmr++) {
				tmp = *(p + nc * lmr);
				maxima = (tmp > maxima) ? tmp : maxima;
			}
			maxMc = (maxima > maxMc) ? maxima : maxMc;
			avgDxy[0][r+c] = (maxima == avgDxy[0][r+c]) ? maxima : 0;
			data.u8TempImage[THRESHOLD][r+c] = (uint8)  (uint8) MIN(255, MAX(0, (maxima >> 10)));
		}
	}
	OscLog(INFO, "maxMc = %d\n", maxMc);
}

void drawMaxMc() {
	int c, r, x1, y1;
	int dim = locMaxDimension / 2;
	int threshold = (data.ipc.state.nThreshold * maxMc) / 100;
	OscLog(INFO, "threshold = %d\n", threshold);
	// OscLog(INFO, "nThreshold = %d\n", data.ipc.state.nThreshold);

	for(r = nc * (dim + Border+1); r < nr * nc -(dim + Border+1) * nc; r+=nc){
		for(c = dim + Border+1;c < nc - (dim + Border+1); c++) {
			if (avgDxy[0][c + r] >= threshold) {
				x1 = c;
				y1 = r / nc;
				DrawBoundingBox(x1-SizeBox, y1+SizeBox, x1+SizeBox, y1-SizeBox, false, GREEN);
			}
		}
	}
}



