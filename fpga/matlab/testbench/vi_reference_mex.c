/* vi_reference_mex.c — MEX gateway for vi_reference_run().
 * Build: mex vi_reference_mex.c ../../host/src/vi_reference_c.c
 *        -I../../host/src -I../../driver/uio
 */
#include "mex.h"
#include "vi_reference_c.h"

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    /* Inputs: value(uint16), penalty(uint16), trans(uint32),
     *         map_x(double), map_y(double), threshold(double), max_sweeps(double) */
    if (nrhs != 7)
        mexErrMsgIdAndTxt("vi:nrhs", "Seven inputs required.");

    uint16_t *value   = (uint16_t *)mxGetData(prhs[0]);
    uint16_t *penalty = (uint16_t *)mxGetData(prhs[1]);
    uint32_t *trans   = (uint32_t *)mxGetData(prhs[2]);
    int map_x      = (int)mxGetScalar(prhs[3]);
    int map_y      = (int)mxGetScalar(prhs[4]);
    uint16_t threshold = (uint16_t)mxGetScalar(prhs[5]);
    int max_sweeps = (int)mxGetScalar(prhs[6]);

    /* Copy value array (reference modifies in-place) */
    mwSize nval = mxGetNumberOfElements(prhs[0]);
    plhs[0] = mxCreateNumericMatrix(1, nval, mxUINT16_CLASS, mxREAL);
    uint16_t *out = (uint16_t *)mxGetData(plhs[0]);
    memcpy(out, value, nval * sizeof(uint16_t));

    int sweeps = vi_reference_run(out, penalty, trans,
                                  map_x, map_y, threshold, max_sweeps);

    /* Return sweep count */
    plhs[1] = mxCreateDoubleScalar((double)sweeps);
}
