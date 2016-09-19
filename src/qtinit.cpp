#include "qtvisualizer.h"

#include <QApplication>

int qtinit(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qtvisualizer w;
    w.show();

    return a.exec();
}
