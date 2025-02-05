/**
 * @file
 * @copyright 2025 Embeint Inc
 * @author Jordan Yates <jordan@embeint.com>
 */

#include <stdint.h>

#include <tfm_ioctl_api.h>

enum tfm_platform_err_t tfm_platform_fault_info_read(struct fault_exception_info_t *destination,
						     uint32_t *result)
{
	enum tfm_platform_err_t ret;
	psa_invec in_vec;
	psa_outvec out_vec;
	struct tfm_fault_info_service_args args;
	struct tfm_fault_info_service_out out;

	args.destination = destination;

	in_vec.base = (const void *)&args;
	in_vec.len = sizeof(args);

	out_vec.base = (void *)&out;
	out_vec.len = sizeof(out);

	ret = tfm_platform_ioctl(TFM_PLATFORM_IOCTL_FAULT_INFO_SERVICE, &in_vec, &out_vec);

	*result = out.result;

	return ret;
}
