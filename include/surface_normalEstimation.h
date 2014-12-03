#ifndef _SURFACE_NORMALESTIMATION_PLUGIN_H_
#define _SURFACE_NORMALESTIMATION_PLUGIN_H_

#include "plugin_interaction.h"

#include "camera.h"

#include "mapHandler.h"

#include "dialog_surface_normalEstimation.h"

namespace CGoGN
{

namespace SCHNApps
{

class Surface_NormalEstimation_Plugin : public PluginInteraction
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

    virtual void draw(View* view);
    virtual void drawMap(View* view, MapHandlerGen* map) {
    }

    virtual void keyPress(View* view, QKeyEvent* event){
    }
    virtual void keyRelease(View* view, QKeyEvent* event) {
    }
    virtual void mousePress(View* view, QMouseEvent* event){
    }
    virtual void mouseRelease(View* view, QMouseEvent* event) {
    }
    virtual void mouseMove(View* view, QMouseEvent* event) {
    }
    virtual void wheelEvent(View* view, QWheelEvent* event){
    }
    virtual void viewLinked(View* view) {
    }
    virtual void viewUnlinked(View* view) {
    }

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
