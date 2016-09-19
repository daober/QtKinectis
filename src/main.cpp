#include "qtvisualizer.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qtvisualizer w;
    w.show();

    return a.exec();
}
