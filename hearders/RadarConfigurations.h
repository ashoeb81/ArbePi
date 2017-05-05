/*
 * Radar sensitivity configurations.
 */

#ifndef ARBEROBOTICSRADARAPI_RADARCONFIGURATIONS_H_
#define ARBEROBOTICSRADARAPI_RADARCONFIGURATIONS_H_


struct Symmetric_Block_Params {
    uint16_t T_sweep;
    uint32_t co_avg_points;
    uint32_t non_co_avg_sweeps;
};
struct RadarConfiguration {
    uint32_t sector_id;
    bool continues;
    uint32_t delayPoints;
    bool with_calib;
    OUTPUT_TYPE output_type;
    uint32_t num_of_blocks;
    const Symmetric_Block_Params *block_params;
};

namespace RadarConfigurations {

    const Symmetric_Block_Params default_blocks[2] = {{1000, 1, 4},
                                                      {2000, 2, 2}};
    const RadarConfiguration DEFAULT_CONFIG = {
            8, // all sectors //uint32_t sector_id;
            true, //bool continues;
            60, //uint32_t delayPoints;
            true, //bool with_calib;
            Targets_Output, //OUTPUT_TYPE output_type;
            2, //uint32_t num_of_blocks
            default_blocks//Symmetric_Block_Params* block_params;
    };


    const Symmetric_Block_Params high_confidence_blocks[2] = {{2000, 2, 10},
                                                              {8000, 8, 5}};
    const RadarConfiguration HIGH_CONFIDANCE_CONFIG = {
            8, // all sectors //uint32_t sector_id;
            true, //bool continues;
            60, //uint32_t delayPoints;
            true, //bool with_calib;
            Targets_Output, //OUTPUT_TYPE output_type;
            2, //uint32_t num_of_blocks
            high_confidence_blocks//Symmetric_Block_Params* block_params;
    };
    const Symmetric_Block_Params high_fps_blocks[1] = {{1000, 1, 1}};
    const RadarConfiguration HIGH_FPS_CONFIG = {
            8, // all sectors //uint32_t sector_id;
            true, //bool continues;
            60, //uint32_t delayPoints;
            true, //bool with_calib;
            Targets_Output, //OUTPUT_TYPE output_type;
            1, //uint32_t num_of_blocks
            high_fps_blocks//Symmetric_Block_Params* block_params;
    };
    //////////////////////////
    //////////////////////////
    //////////////////////////
    const RadarConfiguration DEFAULT_CONFIG_FRONT_SECTOR = {
            3, // front sector  //uint32_t sector_id;
            true, //bool continues;
            60, //uint32_t delayPoints;
            true, //bool with_calib;
            Targets_Output, //OUTPUT_TYPE output_type;
            2, //uint32_t num_of_blocks
            default_blocks//Symmetric_Block_Params* block_params;
    };


    const RadarConfiguration HIGH_CONFIDANCE_CONFIG_FRONT_SECTOR = {
            3, // front sector //uint32_t sector_id;
            true, //bool continues;
            60, //uint32_t delayPoints;
            true, //bool with_calib;
            Targets_Output, //OUTPUT_TYPE output_type;
            2, //uint32_t num_of_blocks
            high_confidence_blocks//Symmetric_Block_Params* block_params;
    };
    const RadarConfiguration HIGH_FPS_CONFIG_FRONT_SECTOR = {
            3, // front sector  //uint32_t sector_id;
            true, //bool continues;
            60, //uint32_t delayPoints;
            true, //bool with_calib;
            Targets_Output, //OUTPUT_TYPE output_type;
            1, //uint32_t num_of_blocks
            high_fps_blocks//Symmetric_Block_Params* block_params;
    };

}


#endif /* ARBEROBOTICSRADARAPI_RADARCONFIGURATIONS_H_ */
