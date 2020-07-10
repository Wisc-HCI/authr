/* -*- Mode: C; c-basic-offset:4 ; indent-tabs-mode:nil -*- */
/*
 * Copyright (c) 2004-2007 The Trustees of Indiana University and Indiana
 *                         University Research and Technology
 *                         Corporation.  All rights reserved.
 * Copyright (c) 2004-2005 The University of Tennessee and The University
 *                         of Tennessee Research Foundation.  All rights
 *                         reserved.
 * Copyright (c) 2004-2005 High Performance Computing Center Stuttgart,
 *                         University of Stuttgart.  All rights reserved.
 * Copyright (c) 2004-2005 The Regents of the University of California.
 *                         All rights reserved.
 * Copyright (c) 2014-2018 Los Alamos National Security, LLC. All rights
 *                         reserved.
 * Copyright (c) 2018      Intel, Inc, All rights reserved
 *
 * $COPYRIGHT$
 *
 * Additional copyrights may follow
 *
 * $HEADER$
 */


#include "opal_config.h"

#include "opal/mca/btl/btl.h"
#include "opal/mca/btl/base/base.h"
#include "opal/mca/hwloc/base/base.h"

#include <string.h>

#include "btl_ofi.h"
#include "btl_ofi_endpoint.h"
#include "btl_ofi_rdma.h"

#define MCA_BTL_OFI_REQUIRED_CAPS       (FI_RMA | FI_ATOMIC)
#define MCA_BTL_OFI_REQUESTED_MR_MODE   (FI_MR_ALLOCATED | FI_MR_PROV_KEY | FI_MR_VIRT_ADDR)

static char *prov_include;
static char *prov_exclude;
static char *ofi_progress_mode;
static bool disable_sep;
static int mca_btl_ofi_init_device(struct fi_info *info);

/* validate information returned from fi_getinfo().
 * return OPAL_ERROR if we dont have what we need. */
static int validate_info(struct fi_info *info)
{
    int mr_mode;

    BTL_VERBOSE(("validating device: %s", info->domain_attr->name));

    /* we need exactly all the required bits */
    if ((info->caps & MCA_BTL_OFI_REQUIRED_CAPS) != MCA_BTL_OFI_REQUIRED_CAPS) {
        BTL_VERBOSE(("unsupported caps"));
        return OPAL_ERROR;
    }

    /* we need FI_EP_RDM */
    if (info->ep_attr->type != FI_EP_RDM) {
        BTL_VERBOSE(("unsupported EP type"));
        return OPAL_ERROR;
    }

    mr_mode = info->domain_attr->mr_mode;

    if (!(mr_mode == FI_MR_BASIC || mr_mode == FI_MR_SCALABLE ||
         (mr_mode & ~(FI_MR_VIRT_ADDR | FI_MR_ALLOCATED | FI_MR_PROV_KEY)) == 0)) {
        BTL_VERBOSE(("unsupported MR mode"));
        return OPAL_ERROR;
    }

    if (!(info->tx_attr->op_flags | FI_DELIVERY_COMPLETE)) {
        BTL_VERBOSE(("the endpoint tx_ctx does not support FI_DELIVERY_COMPLETE"));
        return OPAL_ERROR;
    }

    BTL_VERBOSE(("device: %s is good to go.", info->domain_attr->name));
    return OPAL_SUCCESS;
}

/* Register the MCA parameters */
static int mca_btl_ofi_component_register(void)
{
    mca_btl_ofi_module_t *module = &mca_btl_ofi_module_template;

    /* fi_getinfo with prov_name == NULL means ALL provider.
     * Since now we are using the first valid info returned, I'm not sure
     * if we need to provide the support for comma limited provider list. */
    prov_include = NULL;
    (void) mca_base_component_var_register(&mca_btl_ofi_component.super.btl_version,
                                          "provider_include",
                                          "OFI provider that ofi btl will query for. This parameter only "
                                          "accept ONE provider name. "
                                          "(e.g., \"psm2\"; an empty value means that all providers will "
                                          "be considered.",
                                          MCA_BASE_VAR_TYPE_STRING, NULL, 0, 0,
                                          OPAL_INFO_LVL_4,
                                          MCA_BASE_VAR_SCOPE_READONLY,
                                          &prov_include);

    /* TODO: this param has not been implemented. Not sure if we need it. " */
    prov_exclude = NULL;
    (void) mca_base_component_var_register(&mca_btl_ofi_component.super.btl_version,
                                          "provider_exclude",
                                          "Comma-delimited list of OFI providers that are not considered for use "
                                          "(default: \"sockets,mxm\"; empty value means that all providers will "
                                          " be considered). "
                                          "Mutually exclusive with btl_ofi_provider_include.",
                                          MCA_BASE_VAR_TYPE_STRING, NULL, 0, 0,
                                          OPAL_INFO_LVL_4,
                                          MCA_BASE_VAR_SCOPE_READONLY,
                                          &prov_exclude);

    mca_btl_ofi_component.num_cqe_read = MCA_BTL_OFI_NUM_CQE_READ;
    (void) mca_base_component_var_register(&mca_btl_ofi_component.super.btl_version,
                                          "num_cq_read",
                                          "Number of completion entries to read from a single cq_read. ",
                                          MCA_BASE_VAR_TYPE_INT, NULL, 0, 0,
                                          OPAL_INFO_LVL_5,
                                          MCA_BASE_VAR_SCOPE_READONLY,
                                          &mca_btl_ofi_component.num_cqe_read);

    ofi_progress_mode = "unspec";
    (void) mca_base_component_var_register(&mca_btl_ofi_component.super.btl_version,
                                          "progress_mode",
                                          "requested provider progress mode. [unspec, auto, manual]"
                                          "(default: unspec)",
                                          MCA_BASE_VAR_TYPE_STRING, NULL, 0, 0,
                                          OPAL_INFO_LVL_5,
                                          MCA_BASE_VAR_SCOPE_READONLY,
                                          &ofi_progress_mode);

    mca_btl_ofi_component.num_contexts_per_module = 1;
    (void) mca_base_component_var_register(&mca_btl_ofi_component.super.btl_version,
                                          "num_contexts_per_module",
                                          "number of communication context per module to create. "
                                          "This should increase multithreaded performance but it is "
                                          "advised that this number should be lower than total cores.",
                                          MCA_BASE_VAR_TYPE_INT, NULL, 0, 0,
                                          OPAL_INFO_LVL_5,
                                          MCA_BASE_VAR_SCOPE_READONLY,
                                          &mca_btl_ofi_component.num_contexts_per_module);

    disable_sep = false;
    (void) mca_base_component_var_register(&mca_btl_ofi_component.super.btl_version,
                                          "disable_sep",
                                          "force btl/ofi to never use scalable endpoint. ",
                                          MCA_BASE_VAR_TYPE_BOOL, NULL, 0, 0,
                                          OPAL_INFO_LVL_5,
                                          MCA_BASE_VAR_SCOPE_READONLY,
                                          &disable_sep);

    mca_btl_ofi_component.progress_threshold = MCA_BTL_OFI_PROGRESS_THRESHOLD;
    (void) mca_base_component_var_register(&mca_btl_ofi_component.super.btl_version,
                                          "progress_threshold",
                                          "number of outstanding operation before btl will progress "
                                          "automatically. Tuning this might improve performance on "
                                          "certain type of application.",
                                          MCA_BASE_VAR_TYPE_INT, NULL, 0, 0,
                                          OPAL_INFO_LVL_5,
                                          MCA_BASE_VAR_SCOPE_READONLY,
                                          &mca_btl_ofi_component.progress_threshold);

    /* for now we want this component to lose to btl/ugni and btl/vader */
    module->super.btl_exclusivity = MCA_BTL_EXCLUSIVITY_HIGH - 50;

    return mca_btl_base_param_register (&mca_btl_ofi_component.super.btl_version,
                                        &module->super);
}

static int mca_btl_ofi_component_open(void)
{
    mca_btl_ofi_component.module_count = 0;
    return OPAL_SUCCESS;
}

/*
 * component cleanup - sanity checking of queue lengths
 */
static int mca_btl_ofi_component_close(void)
{
    /* If we don't sleep, sockets provider freaks out. */
    sleep(1);
    return OPAL_SUCCESS;
}

void mca_btl_ofi_exit(void)
{
    BTL_ERROR(("BTL OFI will now abort."));
    exit(1);
}

/*
 *  OFI component initialization:
 *   read interface list from kernel and compare against component parameters
 *   then create a BTL instance for selected interfaces
 */

static mca_btl_base_module_t **mca_btl_ofi_component_init (int *num_btl_modules, bool enable_progress_threads,
                                                           bool enable_mpi_threads)
{
    /* for this BTL to be useful the interface needs to support RDMA and certain atomic operations */
    int rc;
    uint64_t progress_mode;
    unsigned resource_count = 0;
    struct mca_btl_base_module_t **base_modules;

    BTL_VERBOSE(("initializing ofi btl"));

    /* Set up libfabric hints. */
    uint32_t libfabric_api;
    libfabric_api = fi_version();

    /* bail if OFI version is less than 1.5. */
    if (libfabric_api < FI_VERSION(1, 5)) {
        BTL_VERBOSE(("ofi btl disqualified because OFI version < 1.5."));
        return NULL;
    }

    struct fi_info *info, *info_list;
    struct fi_info hints = {0};
    struct fi_ep_attr ep_attr = {0};
    struct fi_rx_attr rx_attr = {0};
    struct fi_tx_attr tx_attr = {0};
    struct fi_fabric_attr fabric_attr = {0};
    struct fi_domain_attr domain_attr = {0};

    /* Select the provider */
    fabric_attr.prov_name = prov_include;

    domain_attr.mr_mode = MCA_BTL_OFI_REQUESTED_MR_MODE;

    /* message progression mode. */
    if (!strcmp(ofi_progress_mode, "auto")) {
        progress_mode = FI_PROGRESS_AUTO;
    } else if (!strcmp(ofi_progress_mode, "manual")) {
        progress_mode = FI_PROGRESS_MANUAL;
    } else {
        progress_mode = FI_PROGRESS_UNSPEC;
    }

    domain_attr.control_progress = progress_mode;
    domain_attr.data_progress = progress_mode;

    /* select endpoint type */
    ep_attr.type = FI_EP_RDM;

    /* ask for capabilities */
    hints.caps = MCA_BTL_OFI_REQUIRED_CAPS;

    hints.fabric_attr = &fabric_attr;
    hints.domain_attr = &domain_attr;
    hints.ep_attr = &ep_attr;
    hints.tx_attr = &tx_attr;
    hints.rx_attr = &rx_attr;

    /* for now */
    tx_attr.iov_limit = 1;
    rx_attr.iov_limit = 1;

    tx_attr.op_flags = FI_DELIVERY_COMPLETE;

    mca_btl_ofi_component.module_count = 0;

    /* do the query. */
    rc = fi_getinfo(FI_VERSION(1, 5), NULL, NULL, 0, &hints, &info_list);
    if (0 != rc) {
        BTL_VERBOSE(("fi_getinfo failed with code %d: %s",rc, fi_strerror(-rc)));
        return NULL;
    }

    /* count the number of resources/ */
    info = info_list;
    while(info) {
        resource_count++;
        info = info->next;
    }
    BTL_VERBOSE(("ofi btl found %d possible resources.", resource_count));

    info = info_list;

    while(info) {
        rc = validate_info(info);
        if (OPAL_SUCCESS == rc) {
            /* Device passed sanity check, let's make a module.
             * We only pick the first device we found valid */
            rc = mca_btl_ofi_init_device(info);
            if (OPAL_SUCCESS == rc)
                break;
        }
        info = info->next;
    }

    /* We are done with the returned info. */
    fi_freeinfo(info_list);

    /* pass module array back to caller */
    base_modules = calloc (mca_btl_ofi_component.module_count, sizeof (*base_modules));
    if (NULL == base_modules) {
        return NULL;
    }

    memcpy(base_modules, mca_btl_ofi_component.modules,
           mca_btl_ofi_component.module_count *sizeof (mca_btl_ofi_component.modules[0]));

    BTL_VERBOSE(("ofi btl initialization complete. found %d suitable transports",
                 mca_btl_ofi_component.module_count));

    *num_btl_modules = mca_btl_ofi_component.module_count;

    return base_modules;
}

static int mca_btl_ofi_init_device(struct fi_info *info)
{
    int rc;
    int *module_count = &mca_btl_ofi_component.module_count;
    size_t namelen;
    size_t num_contexts_to_create;

    char *linux_device_name;
    char ep_name[FI_NAME_MAX];

    struct fi_info *ofi_info;
    struct fi_ep_attr *ep_attr;
    struct fi_domain_attr *domain_attr;
    struct fi_av_attr av_attr = {0};
    struct fid_fabric *fabric = NULL;
    struct fid_domain *domain = NULL;
    struct fid_ep *ep = NULL;
    struct fid_av *av = NULL;

    mca_btl_ofi_module_t *module;

    /* allocate module */
    module = (mca_btl_ofi_module_t*) calloc(1, sizeof(mca_btl_ofi_module_t));
    if (NULL == module) {
        BTL_ERROR(("failed to allocate memory for OFI module"));
        goto fail;
    }
    *module = mca_btl_ofi_module_template;

    /* make a copy of the given info to store on the module */
    ofi_info = fi_dupinfo(info);
    ep_attr = ofi_info->ep_attr;
    domain_attr = ofi_info->domain_attr;

    linux_device_name = info->domain_attr->name;
    BTL_VERBOSE(("initializing dev:%s provider:%s",
                    linux_device_name,
                    info->fabric_attr->prov_name));

    /* fabric */
    rc = fi_fabric(ofi_info->fabric_attr, &fabric, NULL);
    if (0 != rc) {
        BTL_VERBOSE(("%s failed fi_fabric with err=%s",
                        linux_device_name,
                        fi_strerror(-rc)
                        ));
        goto fail;
    }

    /* domain */
    rc = fi_domain(fabric, ofi_info, &domain, NULL);
    if (0 != rc) {
        BTL_VERBOSE(("%s failed fi_domain with err=%s",
                        linux_device_name,
                        fi_strerror(-rc)
                        ));
        goto fail;
    }

    /* AV */
    av_attr.type = FI_AV_MAP;
    rc = fi_av_open(domain, &av_attr, &av, NULL);
    if (0 != rc) {
        BTL_VERBOSE(("%s failed fi_av_open with err=%s",
                        linux_device_name,
                        fi_strerror(-rc)
                        ));
        goto fail;
    }

    num_contexts_to_create = mca_btl_ofi_component.num_contexts_per_module;

    /* If the domain support scalable endpoint. */
    if (domain_attr->max_ep_tx_ctx > 1 && !disable_sep) {

        BTL_VERBOSE(("btl/ofi using scalable endpoint."));

        if (num_contexts_to_create > domain_attr->max_ep_tx_ctx) {
            BTL_VERBOSE(("cannot create requested %u contexts. (node max=%zu)",
                            module->num_contexts,
                            domain_attr->max_ep_tx_ctx));
            goto fail;
         }

        /* modify the info to let the provider know we are creating x contexts */
        ep_attr->tx_ctx_cnt = num_contexts_to_create;
        ep_attr->rx_ctx_cnt = num_contexts_to_create;

        /* create scalable endpoint */
        rc = fi_scalable_ep(domain, ofi_info, &ep, NULL);
        if (0 != rc) {
            BTL_VERBOSE(("%s failed fi_scalable_ep with err=%s",
                            linux_device_name,
                            fi_strerror(-rc)
                            ));
            goto fail;
        }

        module->num_contexts = num_contexts_to_create;
        module->is_scalable_ep = true;

        /* create contexts */
        module->contexts = mca_btl_ofi_context_alloc_scalable(ofi_info,
                                domain, ep, av,
                                num_contexts_to_create);

   } else {
        /* warn the user if they want more than 1 context */
        if (num_contexts_to_create > 1) {
            BTL_ERROR(("cannot create %zu contexts as the provider does not support "
                        "scalable endpoint. Falling back to single context endpoint.",
                        num_contexts_to_create));
        }

        BTL_VERBOSE(("btl/ofi using normal endpoint."));

        rc = fi_endpoint(domain, ofi_info, &ep, NULL);
        if (0 != rc) {
            BTL_VERBOSE(("%s failed fi_endpoint with err=%s",
                            linux_device_name,
                            fi_strerror(-rc)
                            ));
            goto fail;
        }

        module->num_contexts = 1;
        module->is_scalable_ep = false;

        /* create contexts */
        module->contexts = mca_btl_ofi_context_alloc_normal(ofi_info,
                                                            domain, ep, av);
    }

    if (NULL == module->contexts) {
        /* error message is already printed */
        goto fail;
    }

    /* enable the endpoint for using */
    rc = fi_enable(ep);
    if (0 != rc) {
        BTL_VERBOSE(("%s failed fi_enable with err=%s",
                        linux_device_name,
                        fi_strerror(-rc)
                        ));
        goto fail;
    }

    /* Everything succeeded, lets create a module for this device. */
    /* store the information. */
    module->fabric_info = ofi_info;
    module->fabric = fabric;
    module->domain = domain;
    module->av = av;
    module->ofi_endpoint = ep;
    module->linux_device_name = linux_device_name;
    module->outstanding_rdma = 0;
    module->use_virt_addr = false;

    if (ofi_info->domain_attr->mr_mode == FI_MR_BASIC ||
        ofi_info->domain_attr->mr_mode & FI_MR_VIRT_ADDR) {
        module->use_virt_addr = true;
    }

    /* initialize the rcache */
    mca_btl_ofi_rcache_init(module);

    /* create endpoint list */
    OBJ_CONSTRUCT(&module->endpoints, opal_list_t);
    OBJ_CONSTRUCT(&module->module_lock, opal_mutex_t);

    /* create and send the modex for this device */
    namelen = sizeof(ep_name);
    rc = fi_getname((fid_t)ep, &ep_name[0], &namelen);
    if (0 != rc) {
        BTL_VERBOSE(("%s failed fi_getname with err=%s",
                        linux_device_name,
                        fi_strerror(-rc)
                        ));
        goto fail;
    }

    /* post our endpoint name so peer can use it to connect to us */
    OPAL_MODEX_SEND(rc,
                    OPAL_PMIX_GLOBAL,
                    &mca_btl_ofi_component.super.btl_version,
                    &ep_name,
                    namelen);
    mca_btl_ofi_component.namelen = namelen;

    /* add this module to the list */
    mca_btl_ofi_component.modules[(*module_count)++] = module;

    return OPAL_SUCCESS;

fail:
    /* clean up */

    /* if the contexts have not been initiated, num_contexts should
     * be zero and we skip this. */
    for (int i=0; i < module->num_contexts; i++) {
        mca_btl_ofi_context_finalize(&module->contexts[i], module->is_scalable_ep);
    }
    free(module->contexts);

    if (NULL != av) {
        fi_close(&av->fid);
    }

    if (NULL != ep) {
        fi_close(&ep->fid);
    }

    if (NULL != domain) {
        fi_close(&domain->fid);
    }

    if (NULL != fabric) {
        fi_close(&fabric->fid);
    }
    free(module);

    /* not really a failure. just skip this device. */
    return OPAL_ERR_OUT_OF_RESOURCE;
}

/**
 * @brief OFI BTL progress function
 *
 * This function explictly progresses all workers.
 */
static int mca_btl_ofi_component_progress (void)
{
    int events = 0;
    mca_btl_ofi_context_t *context;

    for (int i = 0 ; i < mca_btl_ofi_component.module_count ; ++i) {
        mca_btl_ofi_module_t *module = mca_btl_ofi_component.modules[i];

        /* progress context we own first. */
        context = get_ofi_context(module);

        if (mca_btl_ofi_context_trylock(context)) {
            events += mca_btl_ofi_context_progress(context);
            mca_btl_ofi_context_unlock(context);
        }

        /* if there is nothing to do, try progress other's. */
        if (events == 0) {
            for (int j = 0 ; j < module->num_contexts ; j++ ) {

                context = get_ofi_context_rr(module);

                if (mca_btl_ofi_context_trylock(context)) {
                    events += mca_btl_ofi_context_progress(context);
                    mca_btl_ofi_context_unlock(context);
                }

                /* If we did something, good enough. return now.
                 * This is crucial for performance/latency. */
                if (events > 0) {
                    break;
                }
            }
        }
    }

    return events;
}

int mca_btl_ofi_context_progress(mca_btl_ofi_context_t *context) {

    int ret = 0;
    int events_read;
    int events = 0;
    struct fi_cq_entry cq_entry[MCA_BTL_OFI_MAX_CQ_READ_ENTRIES];
    struct fi_cq_err_entry cqerr = {0};

    mca_btl_ofi_completion_t *comp;

    ret = fi_cq_read(context->cq, &cq_entry, mca_btl_ofi_component.num_cqe_read);

    if (0 < ret) {
        events_read = ret;
        for (int i = 0; i < events_read; i++) {
            if (NULL != cq_entry[i].op_context) {
                ++events;
                comp = (mca_btl_ofi_completion_t*) cq_entry[i].op_context;
                mca_btl_ofi_module_t *ofi_btl = (mca_btl_ofi_module_t*)comp->btl;

                switch (comp->type) {
                case MCA_BTL_OFI_TYPE_GET:
                case MCA_BTL_OFI_TYPE_PUT:
                case MCA_BTL_OFI_TYPE_AOP:
                case MCA_BTL_OFI_TYPE_AFOP:
                case MCA_BTL_OFI_TYPE_CSWAP:

                    /* call the callback */
                    if (comp->cbfunc) {
                        comp->cbfunc (comp->btl, comp->endpoint,
                                         comp->local_address, comp->local_handle,
                                         comp->cbcontext, comp->cbdata, OPAL_SUCCESS);
                    }

                    /* return the completion handler */
                    opal_free_list_return(comp->my_list, (opal_free_list_item_t*) comp);

                    MCA_BTL_OFI_NUM_RDMA_DEC(ofi_btl);
                    break;

                default:
                    /* catasthrophic */
                    BTL_ERROR(("unknown completion type"));
                    MCA_BTL_OFI_ABORT();
                }
            }
        }
    } else if (OPAL_UNLIKELY(ret == -FI_EAVAIL)) {
        ret = fi_cq_readerr(context->cq, &cqerr, 0);

        /* cq readerr failed!? */
        if (0 > ret) {
            BTL_ERROR(("%s:%d: Error returned from fi_cq_readerr: %s(%d)",
                       __FILE__, __LINE__, fi_strerror(-ret), ret));
        } else {
            BTL_ERROR(("fi_cq_readerr: (provider err_code = %d)\n",
                       cqerr.prov_errno));
        }
        MCA_BTL_OFI_ABORT();
    }
#ifdef FI_EINTR
    /* sometimes, sockets provider complain about interupt. We do nothing. */
    else if (OPAL_UNLIKELY(ret == -FI_EINTR)) {

    }
#endif
    /* If the error is not FI_EAGAIN, report the error and abort. */
    else if (OPAL_UNLIKELY(ret != -FI_EAGAIN)) {
        BTL_ERROR(("fi_cq_read returned error %d:%s", ret, fi_strerror(-ret)));
        MCA_BTL_OFI_ABORT();
    }

    return events;
}

/** OFI btl component */
mca_btl_ofi_component_t mca_btl_ofi_component = {
    .super = {
        .btl_version = {
            MCA_BTL_DEFAULT_VERSION("ofi"),
            .mca_open_component = mca_btl_ofi_component_open,
            .mca_close_component = mca_btl_ofi_component_close,
            .mca_register_component_params = mca_btl_ofi_component_register,
        },
        .btl_data = {
            /* The component is not checkpoint ready */
            .param_field = MCA_BASE_METADATA_PARAM_NONE
        },

        .btl_init = mca_btl_ofi_component_init,
        .btl_progress = mca_btl_ofi_component_progress,
    },
};
