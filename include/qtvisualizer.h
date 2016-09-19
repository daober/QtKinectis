#ifndef QTVISUALIZER_H
#define QTVISUALIZER_H

#include <QMainWindow>

namespace Ui {
class qtvisualizer;
}

class qtvisualizer : public QMainWindow
{
    Q_OBJECT

public:
    explicit qtvisualizer(QWidget *parent = 0);
    ~qtvisualizer();

private:
    Ui::qtvisualizer *ui;
};

#endif // QTVISUALIZER_H
