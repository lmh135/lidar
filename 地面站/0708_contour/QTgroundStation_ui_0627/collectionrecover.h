#ifndef COLLECTIONRECOVER_H
#define COLLECTIONRECOVER_H

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <QObject>
#include <QString>
#include "point_receive.h"

using namespace std;


class CollectionRecover : public QObject
{
    Q_OBJECT
public:
    explicit CollectionRecover(QObject *parent = nullptr);

    Q_point_recv_node recv_node;

    QString fileName;

signals:

public slots:
};

#endif // COLLECTIONRECOVER_H
