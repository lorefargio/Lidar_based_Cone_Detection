#pragma once
#include "utils/types.hpp"

namespace fs_perception {

class EstimatorInterface {
public:
    virtual ~EstimatorInterface() = default;
    
    /**
     * @brief Stima se il cluster è un cono e ne valuta le proprietà.
     * @param cluster PointCloud del singolo oggetto candidato
     * @return Cone contenente confidenza (0.0 = non cono, >0.5 = cono) e posa.
     */
    virtual Cone estimate(const PointCloudPtr& cluster) = 0;
};

} // namespace fs_perception