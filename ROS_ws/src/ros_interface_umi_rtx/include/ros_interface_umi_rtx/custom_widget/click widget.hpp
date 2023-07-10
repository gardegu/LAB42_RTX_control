#ifndef __CUS_WIDGET_HPP__
#define __CUS_WIDGET_HPP__

#include <QApplication>
#include <QMouseEvent>
#include <QLabel>
#include <QWidget>

#include <iostream>

using namespace std;

class ClickWidget : public QWidget
{
    Q_OBJECT
public:
    ClickWidget(QWidget *parent = nullptr) : QWidget(parent) {}

    int x_pix, y_pix;

    void mousePressEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton)
        {
            // Récupérer les coordonnées relatives au widget
            QPoint pos = event->pos();
            x_pix = pos.x();
            y_pix = pos.y();

            // Faire quelque chose avec les coordonnées
            // cout << "Clic détecté aux coordonnées : x=" << x_pix << ", y=" << y_pix << endl;
        }
        QWidget::mousePressEvent(event);
    }
protected:
};





#endif