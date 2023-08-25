compile:
	$(CC) -I$(VIA_LKL_PATH)/usr/include/ -I$(VIA_LKL_PATH)/tools/lkl/include -o via_test_harness -L$(VIA_LKL_PATH)/tools/lkl/lib  -llkl -lpthread -lm -lrt -ldl -fsanitize=fuzzer-no-link -fsanitize=address -fsanitize-recover=address via_test_harness.c
	$(CC) -I$(VIA_LKL_PATH)/usr/include/ -I$(VIA_LKL_PATH)/tools/lkl/include -o via_uhci_harness -L$(VIA_LKL_PATH)/tools/lkl/lib  -llkl -lpthread -lm -lrt -ldl -fsanitize=fuzzer-no-link -fsanitize=address -fsanitize-recover=address via_uhci_harness.c

disas:
	$(CC) -S -I$(VIA_LKL_PATH)/usr/include/ -I$(VIA_LKL_PATH)/tools/lkl/include -o via_uhci_harness.dis -L$(VIA_LKL_PATH)/tools/lkl/lib  -llkl -lpthread -lm -lrt -ldl -fsanitize=fuzzer-no-link -fsanitize=address -fsanitize-recover=address via_uhci_harness.c
    
