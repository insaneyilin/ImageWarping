// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: main.cpp

#include <QtWidgets/QApplication>
#include "mainwindow.h"

int main(int argc, char **argv) {
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
