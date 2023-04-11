#include <cmath>
#include <iomanip>

#include "Filter.h"

Filter::Filter(sc_module_name n)
    : sc_module(n), t_skt("t_skt"), base_offset(0) {
  SC_THREAD(do_filter);

  t_skt.register_b_transport(this, &Filter::blocking_transport);
}

Filter::~Filter() = default;

// mask
// const int mask[MASK_X][MASK_Y] = {{1, 1, 1}, 
//                                   {1, 2, 1}, 
//                                   {1, 1, 1}};

void Filter::do_filter() {
  int rbuffer[2][3] = {0},gbuffer[2][3] = {0},bbuffer[2][3] = {0};
  while (true) {
    
    std::vector<int> reds, greens, blues;
    int red[3][3] = {0},green[3][3] = {0},blue[3][3] = {0};

    val_r = 0;
    val_g = 0;
    val_b = 0;
    unsigned char flag = buffer.read();
    
    if (flag == 1){
      for (int i = 0; i < 8; i ++){
        flag = buffer.read();
      }
    } else {
      for (int i = 0; i < 2; i ++){
        flag = buffer.read();
      }
    }

    if(flag){
      for (unsigned int v = 0; v < MASK_Y; ++v){
        for (unsigned int u = 0; u < MASK_X; ++u){
          red[u][v] = r.read();
          green[u][v] = g.read();
          blue[u][v] = b.read();
        }
      }
    }
    else{
      for (unsigned int v = 0; v < MASK_Y; ++v){
        for (unsigned int u = 0; u < MASK_X; ++u){
          if(u!=2){
          red[u][v] = rbuffer[u][v];
          green[u][v] = gbuffer[u][v];
          blue[u][v] = bbuffer[u][v];
          }
          else{
          red[u][v] = r.read();
          green[u][v] = g.read();
          blue[u][v] = b.read();
          }
        }
      }
    }

    for (unsigned int v = 0; v < MASK_Y; ++v){
      for (unsigned int u = 0; u < MASK_X; ++u){
        val_r += red[u][v] * mask[u][v];
        reds.push_back(red[u][v]);
        val_g += green[u][v] * mask[u][v];
        greens.push_back(green[u][v]);
        val_b += blue[u][v] * mask[u][v];
        blues.push_back(blue[u][v]);
      }
    }
    
    std::sort(reds.begin(), reds.begin() + MASK_Y * MASK_X);
    std::sort(greens.begin(), greens.begin() + MASK_Y * MASK_X);
    std::sort(blues.begin(), blues.begin() + MASK_Y * MASK_X);

    val_r -= (reds[8]);
    val_g -= (greens[8]);
    val_b -= (blues[8]);
    result_r.write((val_r)/9);
    result_g.write((val_g)/9);
    result_b.write((val_b)/9);

    for (unsigned int v = 0; v < MASK_Y; ++v){
      for (unsigned int u = 0; u < MASK_X-1; ++u){ 
      rbuffer[u][v] = red[u+1][v];
      gbuffer[u][v] = green[u+1][v]; 
      bbuffer[u][v] = blue[u+1][v];
      }
    }
    
    wait(1 * CLOCK_PERIOD, SC_NS);
  }
}

void Filter::blocking_transport(tlm::tlm_generic_payload &payload,
                                     sc_core::sc_time &delay) {
  sc_dt::uint64 addr = payload.get_address();
  addr = addr - base_offset;
  unsigned char *mask_ptr = payload.get_byte_enable_ptr();
  unsigned char *data_ptr = payload.get_data_ptr();
  word temp;
  switch (payload.get_command()) {
  case tlm::TLM_READ_COMMAND:                  
    switch (addr) {
    case SOBEL_FILTER_RESULT_ADDR:
      temp.uc[0] = result_r.read();
      temp.uc[1] = result_g.read();
      temp.uc[2] = result_b.read();
      temp.uc[3] = 0;
      break;
    case SOBEL_FILTER_CHECK_ADDR:
      temp.uint = result_r.num_available();
      break;
    default:
      std::cerr << "Error! SobelFilter::blocking_transport: address 0x"
                << std::setfill('0') << std::setw(8) << std::hex << addr
                << std::dec << " is not valid" << std::endl;
      break;
    }
    data_ptr[0] = temp.uc[0];
    data_ptr[1] = temp.uc[1];
    data_ptr[2] = temp.uc[2];
    data_ptr[3] = temp.uc[3];
    delay=sc_time(1, SC_NS);
    break;

  case tlm::TLM_WRITE_COMMAND:                 
    switch (addr) {
    case SOBEL_FILTER_R_ADDR:
      if (mask_ptr[0] == 0xff) {
        r.write(data_ptr[0]);
      }
      if (mask_ptr[1] == 0xff) {
        g.write(data_ptr[1]);
      }
      if (mask_ptr[2] == 0xff) {
        b.write(data_ptr[2]);
      }
      if (mask_ptr[3] == 0xff) {
        buffer.write(data_ptr[3]);
      }
      break;
    default:
      std::cerr << "Error! SobelFilter::blocking_transport: address 0x"
                << std::setfill('0') << std::setw(8) << std::hex << addr
                << std::dec << " is not valid" << std::endl;
      break;
    }
    delay=sc_time(1, SC_NS);
    break;

  case tlm::TLM_IGNORE_COMMAND:
    payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
    return;
  default:
    payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
    return;
  }
  payload.set_response_status(tlm::TLM_OK_RESPONSE); // Always OK
}