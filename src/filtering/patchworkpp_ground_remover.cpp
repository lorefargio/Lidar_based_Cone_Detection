#include "filtering/patchworkpp_ground_remover.hpp"
#include <pcl/common/io.h>

namespace fs_perception {

PatchworkppGroundRemover::PatchworkppGroundRemover(const patchwork::Params& params) 
    : params_(params) {
    patchwork_ptr_ = std::make_unique<patchwork::PatchWorkpp>(params_);
}

void PatchworkppGroundRemover::removeGround(const PointCloudConstPtr& cloud_in, 
                                            PointCloudPtr& cloud_obstacles, 
                                            PointCloudPtr& cloud_ground) {
    if (cloud_in->empty() || !patchwork_ptr_) return;

    // 1. Pre-filtraggio e conversione (Usiamo il membro persistente)
    // Ridimensioniamo solo se necessario, per non riallocare se la taglia è simile
    cloud_eigen_.resize(cloud_in->size(), 4);
    
    int valid_count = 0;
    for (const auto& pt : cloud_in->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            // Cast esplicito di intensity a float per sicurezza
            cloud_eigen_.row(valid_count) << (float)pt.x, (float)pt.y, (float)pt.z, (float)pt.intensity;
            valid_count++;
        }
    }

    if (valid_count == 0) return;

    // 2. Esecuzione (Patchwork++ lavora bene su blocchi di dati topRows)
    // Usiamo una View invece di copiare i dati
    patchwork_ptr_->estimateGround(cloud_eigen_.topRows(valid_count));

    // 3. Recupero (Senza distruttori locali pesanti)
    ground_pts_ = patchwork_ptr_->getGround();
    nonground_pts_ = patchwork_ptr_->getNonground();

    // 4. Popolamento rapido
    auto populate = [&](const Eigen::MatrixX3f& src, PointCloudPtr& dst) {
        dst->points.clear();
        dst->points.reserve(src.rows());
        for (int i = 0; i < src.rows(); ++i) {
            PointT p;
            p.x = src(i, 0); p.y = src(i, 1); p.z = src(i, 2);
            p.intensity = src(i,3);
            dst->push_back(p);
        }
        dst->width = dst->points.size();
        dst->height = 1;
        dst->is_dense = true;
        dst->header = cloud_in->header;
    };

    populate(ground_pts_, cloud_ground);
    populate(nonground_pts_, cloud_obstacles);
}

} // namespace fs_perception
