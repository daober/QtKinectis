#ifndef QTVISUALIZER_H
#define QTVISUALIZER_H

#include <QWidget>

namespace Ui {
class qtvisualizer;
}

class qtvisualizer : public QWidget
{
    Q_OBJECT

public:
    explicit qtvisualizer(QWidget *parent = 0);
    ~qtvisualizer();

private:
    Ui::qtvisualizer *ui;
};

#endif // QTVISUALIZER_H
