/*
 * Copyright (C) 2014 EPFL
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/.
 */

/**
 * @file MatrixTransform3D.cpp
 * @brief Transforms QtQuick3D items by QMatrix4x4 transform matrix
 * @author Ayberk Özgür
 * @version 1.0
 * @date 2014-10-12
 */

#include "MatrixTransform3D.h"

MatrixTransform3D::MatrixTransform3D(QObject* parent) :
    QQuickQGraphicsTransform3D(parent)
{
    //We're basically overriding the signal of our parent with our own since we can't NOTIFY with our parent's signal
    connect(this, &QQuickQGraphicsTransform3D::transformChanged, this, &MatrixTransform3D::transformChanged);
}

QMatrix4x4 MatrixTransform3D::getMatrix() const
{
    return matrix;
}

void MatrixTransform3D::setMatrix(QMatrix4x4 matrix)
{
    if (this->matrix == matrix)
        return;
    this->matrix = matrix;
    emit transformChanged();
}

void MatrixTransform3D::applyTo(QMatrix4x4* matrix) const
{
    matrix->operator*=(this->matrix);
}

QQuickQGraphicsTransform3D* MatrixTransform3D::clone(QObject* parent) const
{
    MatrixTransform3D* copy = new MatrixTransform3D(parent);
    copy->setMatrix(this->matrix);
    return copy;
}

