#ifndef _SURFACE_NORMALESTIMATION_PLUGIN_H_
#define _SURFACE_NORMALESTIMATION_PLUGIN_H_

#include "plugin_processing.h"

#include "camera.h"

#include "dialog_surface_normalEstimation.h"

namespace CGoGN
{

namespace SCHNApps
{

class Surface_NormalEstimation_Plugin : public Plugin
{
    Q_OBJECT
    Q_INTERFACES(CGoGN::SCHNApps::Plugin)

public:
    Surface_NormalEstimation_Plugin()
    {}

    ~Surface_NormalEstimation_Plugin()
    {}

    virtual bool enable();
    virtual void disable();

private slots:
    void openNormalEstimationDialog();
    void closeNormalEstimationDialog();

    void estimateFromDialog();

public slots:

private:
    Dialog_Surface_NormalEstimation* m_normalEstimationDialog;

    QAction* m_normalEstimationAction;
};

} // namespace SCHNApps

} // namespace CGoGN

#endif
