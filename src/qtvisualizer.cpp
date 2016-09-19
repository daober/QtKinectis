#include "qtvisualizer.h"
#include "ui_qtvisualizer.h"

qtvisualizer::qtvisualizer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::qtvisualizer)
{
    ui->setupUi(this);
}

qtvisualizer::~qtvisualizer()
{
    delete ui;
}
