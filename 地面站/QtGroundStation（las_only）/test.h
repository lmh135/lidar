#ifndef TEST_H
#define TEST_H

#include <QMainWindow>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include "select.h"
#include <liblas/liblas.hpp>
#include <liblas/header.hpp>
#include <fstream>
#include <sstream>
#include <QKeyEvent>
#include "data_construct.h"


namespace Ui {
class test;
}

class test : public QMainWindow
{
    Q_OBJECT

public:
    explicit test(QWidget *parent = 0);
    ~test();

    QPoint press_Pos;
    QPoint release_Pos;

    void DrawPoint();
    bool loadlasFile(const QString &fileName); // 加载文件



private slots:
    void on_return_Btn_clicked();

    void on_init_Btn_clicked();

    void on_pick_Btn_clicked();

    void SelectPoint();

    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);

private:
    Ui::test *ui;

    ifstream ifs;
};

extern vector<int> selected_index;
extern vector<Point> selected_point_save;

extern int selectMode;
extern double vorxba,voryba,vorzba;
#endif // TEST_H
