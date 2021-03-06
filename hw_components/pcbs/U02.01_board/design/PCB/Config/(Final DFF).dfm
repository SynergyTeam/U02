.FILETYPE DRC_DFF_FILE
.VERSION "01.00"
.CREATOR MENTOR

.UNITS MM

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
! Batch DFF dialog settings !
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
.SCHEME_NAME "(Final DFF)"
.SCHEME_LOCATION JOB

.CHECK_DRC_DFF YES
.USE_DRC_WINDOWS NO
.INCLUDE_RULE_AREAS NO
.E2E_HZD_LIMIT 10000
.SAMENET_HZD_LIMIT 10000

.SIGNAL_PROXIMITY NO
.SIGNAL_TRACE_WIDTHS NO
.SIGNAL_TRACE_LENGTHS NO
.SIGNAL_LENGTH_TO_WIDTH_RATIO NO
.SIGNAL_TEXT_STROKE_WIDTH NO
.SIGNAL_PAD_ENTRY_SLVR NO
.SIGNAL_PLANE_RATIO_BASED_SLVR NO
.SIGNAL_PLANE_ACUTE_ANGLE_SLVR NO
.SIGNAL_PLANE_PAD_TO_PAD_SLVR NO
.PLANE_PROXIMITY NO
.PLANE_THERMALS_ALL NO
.PLANE_THERMALS_TIE_LEGS NO
.DRILL_PROXIMITY NO
.DRILL_ANNULAR_RING NO
.DRILL_MINIMUM_HOLE_SIZE NO
.DRILL_HOLE_REGISTRATION NO
.SOLDERMASK_PROXIMITY NO
.SOLDERMASK_CLEARANCE NO
.SOLDERMASK_COVERAGE NO
.SOLDERMASK_EXPOSURE NO
.SOLDERMASK_RATIO_BASED_SLVR NO
.SILKSCREEN_PROXIMITY NO
.SILKSCREEN_SIZES NO
.SOLDERPASTE_PROXIMITY NO

.LAYERS_TO_CHECK 

!!!!!!!!!!!!!!!!!!!!!!!!!!!!
! DFF Rules and Clearances !
!!!!!!!!!!!!!!!!!!!!!!!!!!!!
.TRACE_TO_TRACE NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.TRACE_TO_PAD NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.PAD_TO_PAD NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.TRACE_WIDTHS NO
..LAYER 1
...MINIMUM 0.254
...MAXIMUM 0.635
..LAYER 2
...MINIMUM 0.254
...MAXIMUM 0.635

.TRACE_MINIMUM_LENGTHS NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.TRACE_LENGTH_TO_WIDTH_RATIOS NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.SLIVERS_PAD_ENTRY NO
..IGNORE_TEARDROPS NO
..LAYER 1
...ANGLE 4.5466
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...ANGLE 4.5466
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.SLIVERS_RATIO_BASED NO
..LAYER 1
...RATIO 0.0762
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...RATIO 0.0762
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.SLIVERS_ACUTE_ANGLE NO
..LAYER 1
...CLEARANCE 0.0508
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...CLEARANCE 0.0508
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.SLIVERS_PAD_TO_PAD NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.SLIVERS_PAD_TO_VIA NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.SLIVERS_VIA_TO_VIA NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.SILKSCREEN_TO_SMD_PAD_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_SMD_PAD_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_MISC_PAD_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_MISC_PAD_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_PTH_PAD_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_PTH_PAD_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_NPTH_PAD_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_NPTH_PAD_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_PTH_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_PTH_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_NPTH_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_NPTH_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_VIA_PAD_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_VIA_PAD_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_VIA_HOLE_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_VIA_HOLE_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_CONTOUR_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_CONTOUR_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_SOLDERMASK_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_TO_SOLDERMASK_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SILKSCREEN_LINE_WIDTHS_TOP NO
..MINIMUM 0.0762
..MAXIMUM 0.1016

.SILKSCREEN_LINE_WIDTHS_BTM NO
..MINIMUM 0.0762
..MAXIMUM 0.1016

.PLANE_TO_PLANE NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.PAD_THERMAL_SPOKES NO
..2_SPOKE 1
..4_SPOKE 1

.ALL_THERMAL_SPOKES NO

.HOLE_TO_HOLE NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.PTH_TO_CONDUCTIVE NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.NPTH_TO_CONDUCTIVE NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.NPTH_TO_TRACE NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.NPTH_TO_CONTOUR NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.NPTH_TO_PAD_ANNULAR NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.PTH_TO_PAD_ANNULAR NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.VIA_TO_PAD_ANNULAR NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.PTH_REGISTRATION NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.NPTH_REGISTRATION NO
..LAYER 1
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127
..LAYER 2
...SEVERE 0.0762
...MODERATE 0.1016
...WARNING 0.127

.SOLDERMASK_TO_SOLDERMASK_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SOLDERMASK_TO_SOLDERMASK_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SOLDERMASK_TO_CONDUCTIVE_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SOLDERMASK_TO_CONDUCTIVE_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SMD_PAD_TO_MASK_ANNULAR_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SMD_PAD_TO_MASK_ANNULAR_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.MISC_PAD_TO_MASK_ANNULAR_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.MISC_PAD_TO_MASK_ANNULAR_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.PTH_PAD_TO_MASK_ANNULAR_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.PTH_PAD_TO_MASK_ANNULAR_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.NPTH_PAD_TO_MASK_ANNULAR_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.NPTH_PAD_TO_MASK_ANNULAR_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.PTH_TO_MASK_ANNULAR_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.PTH_TO_MASK_ANNULAR_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.NPTH_TO_MASK_ANNULAR_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.NPTH_TO_MASK_ANNULAR_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.MASK_TO_SMD_PAD_ANNULAR_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.MASK_TO_SMD_PAD_ANNULAR_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.MASK_TO_MISC_PAD_ANNULAR_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.MASK_TO_MISC_PAD_ANNULAR_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.MASK_TO_PTH_PAD_ANNULAR_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.MASK_TO_PTH_PAD_ANNULAR_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.EXPOSED_PAD_TO_PAD_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.EXPOSED_PAD_TO_PAD_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.EXPOSED_PAD_TO_NONPAD_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.EXPOSED_PAD_TO_NONPAD_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.EXPOSED_NONPAD_TO_NONPAD_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.EXPOSED_NONPAD_TO_NONPAD_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SLIVERS_RATIO_BASED_MASK_TOP NO
..RATIO 0.0762
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.SLIVERS_RATIO_BASED_MASK_BTM NO
..RATIO 0.0762
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.PASTE_TO_EXPOSED_CU_PAD_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.PASTE_TO_EXPOSED_CU_PAD_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.PASTE_PAD_TO_PASTE_PAD_TOP NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

.PASTE_PAD_TO_PASTE_PAD_BTM NO
..SEVERE 0.0762
..MODERATE 0.1016
..WARNING 0.127

! End Batch DFF Dialog
