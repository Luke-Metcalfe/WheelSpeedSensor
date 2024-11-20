Stripped down to only one channel pair, and named generically (no WSS).
First implementation of a "sync manager", which really just starts transmission of both channels as close as possible.

Not these changes only work on ESP32. ESP32-S3 has a different hal/rmt_ll.h structure, and variables are set differently.
Can implement similar functionality in ESP32-S3 by instead controlling rmt_ll_tx_start(hal->regs, channel_id);

CHANGES:
Added entire "driver" folder as a component (overwrites default folders), and modified rmt_tx.c as follows:

Commented line 642 and 696 in order to disable transmission start:
   //rmt_ll_tx_enable_loop(hal->regs, channel_id, t->loop_count != 0);
   //rmt_ll_tx_start(hal->regs, channel_id);
This means that each RMT channel is setup but not started.

tx_start controls start if signal is not in looping mode. But if in looping mode, tx_enable_loop will trigger start even if tx_start not called.

Created new function to manually control start of each channel transmission, as close as possible to one another:

   void rmt_tx_start(rmt_channel_handle_t channel1, rmt_channel_handle_t channel2)
   {   
      rmt_tx_channel_t *tx_chan1 = __containerof(channel1, rmt_tx_channel_t, base);
      rmt_tx_channel_t *tx_chan2 = __containerof(channel2, rmt_tx_channel_t, base);

      rmt_channel_t *tx_channel1 = &tx_chan1->base;
      rmt_channel_t *tx_channel2 = &tx_chan2->base;

      rmt_group_t *group1 = tx_channel1->group;
      rmt_group_t *group2 = tx_channel2->group;
      
      rmt_hal_context_t *hal1 = &group1->hal;
      rmt_hal_context_t *hal2 = &group2->hal;

      int channel_id1 = tx_channel1->channel_id;
      int channel_id2 = tx_channel2->channel_id;

      rmt_ll_tx_start(hal1->regs, channel_id1); 
      rmt_ll_tx_start(hal2->regs, channel_id2); 
      
      rmt_ll_tx_enable_loop(hal2->regs, channel_id2, true);
      rmt_ll_tx_enable_loop(hal1->regs, channel_id1, true);

   }

Also modified rmt_tx.h by declaring:
   void rmt_tx_start(rmt_channel_handle_t channel1, rmt_channel_handle_t channel2);