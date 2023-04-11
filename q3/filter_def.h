#ifndef FILTER_DEF_H_
#define FILTER_DEF_H_

// sobel mask parameters
const int MASK_N = 2;
const int MASK_X = 3;
const int MASK_Y = 3;

// Sobel Filter inner transport addresses
// Used between blocking_transport() & do_filter()
const int SOBEL_FILTER_R_ADDR = 0x00000000;
const int SOBEL_FILTER_RESULT_ADDR = 0x00000004;
const int SOBEL_FILTER_CHECK_ADDR = 0x00000008;

union word {
  int sint;
  unsigned int uint;
  unsigned char uc[4];
};

// mask
const int mask[MASK_X][MASK_Y] = {{1, 1, 1}, 
                                  {1, 2, 1}, 
                                  {1, 1, 1}};
#endif
