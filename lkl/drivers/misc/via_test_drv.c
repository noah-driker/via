#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>

// lkl_fuzz_platform_dev_config

struct resource* res;
void __iomem *base;


static int via_test_probe(struct platform_device *pdev) {
	pr_info("#### %s() ####\n", __func__);
	printk("%s", "driver is probed");

	// platform_get_resource
	// - gets information on the structure of the device resource
	// - https://stackoverflow.com/questions/22961714/what-is-platform-get-resource-in-linux-driver
	// - last parameter designates which resource of that type is desired (does this connect with n_mmio?)
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	base = devm_ioremap_resource(&pdev->dev, res);


	printk("via test base: %p\n", base);

	printk("test data %d\n", readb((char*)base));

	if (IS_ERR(base)) {
		return PTR_ERR(base);
	}


	return 0;
}

static int via_test_remove(struct platform_device *pdev) {
	pr_info("#### %s ####\n", __func__);
	return 0;
}

static const struct of_device_id via_test_of_id_table[] = {
	{.compatible = "test,via-test-dev"},
	{}

};
MODULE_DEVICE_TABLE(of, via_test_of_id_table);

static struct platform_driver via_test_driver = {
	.driver = {
		.name = "via-test-drv",
		.of_match_table = via_test_of_id_table,
	},
	.probe = via_test_probe,
	.remove = via_test_remove,

};

static int __init via_test_init(void) {
	printk("via test init\n");
	return platform_driver_register(&via_test_driver);
}

static void __exit via_test_exit(void) {
	printk("via test exit\n");
	platform_driver_unregister(&via_test_driver);
}


//module_platform_driver(via_test_driver);

module_init(via_test_init);
module_exit(via_test_exit);

MODULE_AUTHOR("Noah Driker");
MODULE_DESCRIPTION("Via Test Driver");
MODULE_LICENSE("GPL v2");
