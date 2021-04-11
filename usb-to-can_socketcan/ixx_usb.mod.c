#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(.gnu.linkonce.this_module) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section(__versions) = {
	{ 0xc79d2779, "module_layout" },
	{ 0xad436fa8, "netdev_info" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xf9a482f9, "msleep" },
	{ 0x1faa96a9, "register_candev" },
	{ 0x9b482a91, "alloc_can_err_skb" },
	{ 0x9a1fc4b4, "jiffies_to_timeval" },
	{ 0x16081ffb, "can_dlc2len" },
	{ 0x324d0695, "usb_reset_configuration" },
	{ 0xe12328d2, "driver_for_each_device" },
	{ 0x4f38cf60, "kthread_create_on_node" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x73d7a771, "__dynamic_netdev_dbg" },
	{ 0x3024bcaa, "usb_unanchor_urb" },
	{ 0xf6149e67, "can_bus_off" },
	{ 0xebddaad5, "netif_rx" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x4ff50938, "close_candev" },
	{ 0xd885c62a, "netif_tx_wake_queue" },
	{ 0xa2bf42c9, "usb_deregister" },
	{ 0xc5850110, "printk" },
	{ 0x7d5a7d0b, "kthread_stop" },
	{ 0xa1c76e0a, "_cond_resched" },
	{ 0x9166fada, "strncpy" },
	{ 0x843526fb, "usb_control_msg" },
	{ 0xee8e0fb5, "alloc_candev_mqs" },
	{ 0x6a73c3a2, "free_candev" },
	{ 0x9956e5a4, "_dev_err" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x167c5967, "print_hex_dump" },
	{ 0xd6dd5cd0, "can_change_mtu" },
	{ 0xb4055ce3, "usb_submit_urb" },
	{ 0xb601be4c, "__x86_indirect_thunk_rdx" },
	{ 0xc0083fcb, "netif_device_detach" },
	{ 0x962c8ae1, "usb_kill_anchored_urbs" },
	{ 0xb6327b0f, "alloc_can_skb" },
	{ 0xdecd0b29, "__stack_chk_fail" },
	{ 0x1000e51, "schedule" },
	{ 0xce58ba8, "kfree_skb" },
	{ 0x2ea2c95c, "__x86_indirect_thunk_rax" },
	{ 0x3129b92e, "wake_up_process" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x7bebc891, "netdev_err" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0xe077cc4d, "open_candev" },
	{ 0x834ea889, "__dynamic_dev_dbg" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0xf6ebc03b, "net_ratelimit" },
	{ 0xb3f7646e, "kthread_should_stop" },
	{ 0xda45c8ea, "netdev_warn" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0x37a0cba, "kfree" },
	{ 0x69acdf38, "memcpy" },
	{ 0xacc38f46, "usb_register_driver" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x820cc3e5, "alloc_canfd_skb" },
	{ 0x1ff06dbb, "unregister_netdev" },
	{ 0x8762619a, "can_len2dlc" },
	{ 0x8a741288, "can_get_echo_skb" },
	{ 0x2e08802e, "consume_skb" },
	{ 0xabb0ed6b, "can_put_echo_skb" },
	{ 0xe3f03a8e, "can_free_echo_skb" },
	{ 0xb9daafa, "usb_free_urb" },
	{ 0x8599c50f, "usb_anchor_urb" },
	{ 0xcb64db4d, "usb_alloc_urb" },
};

MODULE_INFO(depends, "can-dev");

MODULE_ALIAS("usb:v08D8p0008d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p0009d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p000Ad*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p000Bd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p000Cd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p000Dd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p0017d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p0014d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p0016d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p001Bd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p001Cd*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "81195CD103613AF12A0905B");
