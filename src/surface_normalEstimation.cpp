#include "surface_normalEstimation.h"

#include "mapHandler.h"

#define NORMALE 5

namespace CGoGN
{

namespace SCHNApps
{

bool Surface_NormalEstimation_Plugin::enable()
{
    m_normalEstimationDialog = new Dialog_Surface_NormalEstimation(m_schnapps);

    m_normalEstimationAction = new QAction("Normal Estimation", this);

    m_schnapps->addMenuAction(this, "Surface;Normal Estimation", m_normalEstimationAction);

    connect(m_normalEstimationAction, SIGNAL(triggered()), this, SLOT(openNormalEstimationDialog()));

    connect(m_normalEstimationDialog, SIGNAL(accepted()), this, SLOT(estimateFromDialog()));
    connect(m_normalEstimationDialog->button_cancel, SIGNAL(clicked()), this, SLOT(closeNormalEstimationDialog()));
    connect(m_normalEstimationDialog->button_ok, SIGNAL(clicked()), this, SLOT(estimateFromDialog()));

    return true;
}

void Surface_NormalEstimation_Plugin::disable()
{
    disconnect(m_normalEstimationAction, SIGNAL(triggered()), this, SLOT(openNormalEstimationDialog()));

    disconnect(m_normalEstimationDialog, SIGNAL(accepted()), this, SLOT(inverseFromDialog()));
    disconnect(m_normalEstimationDialog->button_cancel, SIGNAL(clicked()), this, SLOT(closeNormalEstimationDialog()));
    disconnect(m_normalEstimationDialog->button_ok, SIGNAL(clicked()), this, SLOT(estimateFromDialog()));
}

void Surface_NormalEstimation_Plugin::draw(View* view)
{
    QList<QListWidgetItem*> currentItems = m_normalEstimationDialog->list_maps->selectedItems();
    if(!currentItems.empty())
    {
        const QString& mapName = currentItems[0]->text();
        MapHandler<PFP2>* mh_map = static_cast<MapHandler<PFP2>*>(m_schnapps->getMap(mapName));
        if(mh_map)
        {
            PFP2::MAP* map = mh_map->getMap();
            VertexAttribute<PFP2::VEC3, PFP2::MAP> normalMap = mh_map->getAttribute<PFP2::VEC3, VERTEX>("normal");

            if(normalMap.isValid())
            {
                //Dessiner les normales en chaque point
                glCallList(NORMALE);
            }
        }
    }
}

void Surface_NormalEstimation_Plugin::openNormalEstimationDialog()
{
    m_normalEstimationDialog->show();
}

void Surface_NormalEstimation_Plugin::closeNormalEstimationDialog()
{
    m_normalEstimationDialog->close();
}

void Surface_NormalEstimation_Plugin::estimateFromDialog()
{
    QList<QListWidgetItem*> currentItems = m_normalEstimationDialog->list_maps->selectedItems();
    if(!currentItems.empty())
    {
        const QString& mapName = currentItems[0]->text();
        const QString& positionName = m_normalEstimationDialog->combo_positionAttribute->currentText();

        MapHandler<PFP2>* mh_map = static_cast<MapHandler<PFP2>*>(m_schnapps->getMap(mapName));

        if(mh_map)
        {
            PFP2::MAP* map = mh_map->getMap();
            VertexAttribute<PFP2::VEC3, PFP2::MAP> positionMap = mh_map->getAttribute<PFP2::VEC3, VERTEX>(positionName);
            VertexAttribute<PFP2::VEC3, PFP2::MAP> normalMap = mh_map->getAttribute<PFP2::VEC3, VERTEX>("normal");
            if(!normalMap.isValid())
            {
                normalMap = mh_map->addAttribute<PFP2::VEC3, VERTEX>("normal");
            }

            glNewList(NORMALE, GL_COMPILE);

            //Recherche des K plus proches voisins
            const int K = 5;

            TraversorV<PFP2::MAP> trav_vert_map(*map);
            for(Dart d = trav_vert_map.begin(); d != trav_vert_map.end(); d = trav_vert_map.next())
            {
                TraversorV<PFP2::MAP> trav_vert_map2(*map);
                std::vector<Dart> nearest_neighbors;
                nearest_neighbors.resize(K, Dart::nil());
                CGoGNout << "Recherche des k plus proches voisins .." << CGoGNflush;
                for(Dart dd = trav_vert_map2.begin(); dd != trav_vert_map2.end(); dd = trav_vert_map2.next())
                {
                    bool stop = false;
                    for(int i = 0; i < K && !stop; ++i)
                    {
                        if(nearest_neighbors[i].isNil())
                        {
                            nearest_neighbors[i] = dd;
                            stop = true;
                        }
                        else
                        {
                            float distance_1 = pow(positionMap[nearest_neighbors[i]][0]-positionMap[d][0],2)+pow(positionMap[nearest_neighbors[i]][1]-positionMap[d][1],2)+pow(positionMap[nearest_neighbors[i]][2]-positionMap[d][2],2);
                            float distance_2 = pow(positionMap[dd][0]-positionMap[d][0],2)+pow(positionMap[dd][1]-positionMap[d][1],2)+pow(positionMap[dd][2]-positionMap[d][2],2);
                            if(distance_2 > distance_1)
                            {
                                for(int j = K-2; j >= i; --j)
                                {
                                    //On décale les valeurs vers la droite (en supprimant le K-ième voisin)
                                    nearest_neighbors[j+1] = nearest_neighbors[j];
                                }
                                nearest_neighbors[i] = dd;
                                stop = true;
                            }
                        }
                    }
                }
                CGoGNout << ".. terminée" << CGoGNendl;
//                for(int i = 0; i < K; ++i)
//                {
//                    CGoGNout << nearest_neighbors[i] << CGoGNflush;
//                    if(i<K-1)
//                    {
//                        CGoGNout << ", " << CGoGNflush;
//                    }
//                }
//                CGoGNout << "]" << CGoGNendl;

                //Calcul de la matrice de covariance
                Eigen::Matrix<PFP2::REAL, K, 3> neighborsPositionEigen;
                neighborsPositionEigen.setZero(K, 3);
                Eigen::Matrix<PFP2::REAL, 3, 3> covarianceMatrixEigen;
                covarianceMatrixEigen.setZero(3, 3);
                Eigen::Matrix<PFP2::REAL, 1, 3> centroid;
                centroid.setZero(1, 3);

                //Calcul du centroide formé par le voisinage de points
                for(int i = 0; i < K; ++i)
                {
                    neighborsPositionEigen(i, 0) = positionMap[nearest_neighbors[i]][0];
                    neighborsPositionEigen(i, 1) = positionMap[nearest_neighbors[i]][1];
                    neighborsPositionEigen(i, 2) = positionMap[nearest_neighbors[i]][2];

                    centroid(0, 0) += neighborsPositionEigen(i, 0);
                    centroid(0, 1) += neighborsPositionEigen(i, 1);
                    centroid(0, 2) += neighborsPositionEigen(i, 2);
                }
                centroid /= K;

                //Calcul de la matrice de covariance
                for(int i = 0; i < K; ++i)
                {
                    covarianceMatrixEigen(0, 0) += pow(neighborsPositionEigen(i, 0) - centroid[0], 2);
                    covarianceMatrixEigen(0, 1) += pow(neighborsPositionEigen(i, 1) - centroid[1], 2);
                    covarianceMatrixEigen(0, 2) += pow(neighborsPositionEigen(i, 2) - centroid[2], 2);
                }

                Eigen::EigenSolver<Eigen::Matrix<PFP2::REAL, 3, 3>> eigen_solver(covarianceMatrixEigen, true);

                normalMap[d] = PFP2::VEC3(eigen_solver.eigenvectors().col(2)(0).real(), eigen_solver.eigenvectors().col(2)(1).real(), eigen_solver.eigenvectors().col(2)(2).real());

                glBegin(GL_LINE);
                glColor3f(1.f, 1.f, 0.f);
                    glVertex3f(normalMap[d][0], normalMap[d][1], normalMap[d][2]);
                    glVertex3f(positionMap[d][0], positionMap[d][1], positionMap[d][2]);
                glEnd();
                glBegin(GL_POINT);
                    glPointSize(10.f);
                    glColor3f(1.f, 0.f, 0.f);
                    glVertex3f(normalMap[d][0], normalMap[d][1], normalMap[d][2]);
                glEnd();
                glColor3f(1.f, 1.f, 1.f);

//                for(int i = 0; i < eigen_solver.eigenvectors().rows(); ++i)
//                {
//                    CGoGNout << eigen_solver.eigenvectors().row(i) << CGoGNflush;
//                    if(i < eigen_solver.eigenvectors().rows()-1)
//                    CGoGNout << " | " << CGoGNflush;
//                }
//                CGoGNout << "-------------------------------" << CGoGNendl;
            }
            glEndList();
            mh_map->notifyAttributeModification(normalMap);
        }
    }
    m_normalEstimationDialog->close();
}

#ifndef DEBUG
Q_EXPORT_PLUGIN2(Surface_NormalEstimation_Plugin, Surface_NormalEstimation_Plugin)
#else
Q_EXPORT_PLUGIN2(Surface_NormalEstimation_PluginD, Surface_NormalEstimation_Plugin)
#endif

} // namespace SCHNApps

} // namespace CGoGN
