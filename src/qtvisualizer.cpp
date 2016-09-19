#include "qtvisualizer.h"
#include "ui_qtvisualizer.h"

qtvisualizer::qtvisualizer(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::qtvisualizer)
{
    ui->setupUi(this);
}

qtvisualizer::~qtvisualizer()
{
    delete ui;
}
