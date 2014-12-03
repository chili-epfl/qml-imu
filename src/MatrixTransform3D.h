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
 * @file MatrixTransform3D.h
 * @brief Transforms QtQuick3D items by QMatrix4x4 transform matrix
 * @author Ayberk Özgür
 * @version 1.0
 * @date 2014-10-12
 */

#ifndef MATRIXTRANSFORM3D_H
#define MATRIXTRANSFORM3D_H

#include <QQuickQGraphicsTransform3D>
#include <QMatrix4x4>

/**
 * @brief Transforms QtQuick3D items by a QMatrix4x4
 *
 * This object actually shouldn't exist, MatrixTransform should be enough. They
 * have the exact same functionality and nearly the exact same API. But
 * since they don't come from the same parent other than QObject, (MatrixTransform
 * coming from QGraphicsTransform while MatrixTransform3D coming from
 * QGraphicsTransform3D), this object can only be applied to QML Item3D's while
 * the other can only be applied to QML Item's. The reverse is not possible. If
 * in the future they are rebased onto a single parent (which should ideally
 * happen), MatrixTransform3D should be removed from existence.
 */
class MatrixTransform3D : public QQuickQGraphicsTransform3D
{
Q_OBJECT
    Q_PROPERTY(QMatrix4x4 matrix READ getMatrix WRITE setMatrix NOTIFY transformChanged)
    Q_CLASSINFO("DefaultProperty", "matrix")

public:

    /**
     * @brief Creates a new QMatrix4x4 transform
     *
     * @param parent Parent of the new transform
     */
    explicit MatrixTransform3D(QObject* parent = 0);

    /**
     * @brief Gets the matrix that describes the transform
     *
     * @return The matrix that describes the transform
     */
    QMatrix4x4 getMatrix() const;

    /**
     * @brief Sets the matrix that describes the transform
     *
     * @param matrix The matrix that describes the transform
     */
    void setMatrix(QMatrix4x4 matrix);

     /**
     * @brief Applies this transform to the given transform from the left
     *
     * @param matrix Transform to apply this transform to
     */
   virtual void applyTo(QMatrix4x4* matrix) const;

   /**
    * @brief Makes an exact copy of this transform
    *
    * @param parent Parent of the potential copy
    *
    * @return Pointer to the allocated exact copy of this transform
    */
    virtual QQuickQGraphicsTransform3D* clone(QObject* parent) const;

signals:

    /**
     * @brief Emitted every time this transform is changed
     */
    void transformChanged();

private:

    /**
     * @brief Matrix that describes this transform
     */
    QMatrix4x4 matrix;
};

#endif // MATRIXTRANSFORM3D_H
