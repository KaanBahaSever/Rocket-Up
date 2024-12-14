
#include "matrix.h"
#include <stdexcept>
#include <cmath>

Matrix::Matrix(int rows, int cols) : rows_(rows), cols_(cols) {
    if (rows <= 0 || cols <= 0) {
        throw std::invalid_argument("Matrix dimensions must be positive");
    }
    data_ = new double[rows * cols]();
}

Matrix::Matrix(const Matrix& other) : rows_(other.rows_), cols_(other.cols_) {
    data_ = new double[rows_ * cols_];
    for (int i = 0; i < rows_ * cols_; i++) {
        data_[i] = other.data_[i];
    }
}

Matrix::~Matrix() {
    delete[] data_;
}

Matrix& Matrix::operator=(const Matrix& other) {
    if (this != &other) {
        delete[] data_;
        rows_ = other.rows_;
        cols_ = other.cols_;
        data_ = new double[rows_ * cols_];
        for (int i = 0; i < rows_ * cols_; i++) {
            data_[i] = other.data_[i];
        }
    }
    return *this;
}

double& Matrix::operator()(int row, int col) {
    if (row < 0 || row >= rows_ || col < 0 || col >= cols_) {
        throw std::out_of_range("Matrix indices out of range");
    }
    return data_[row * cols_ + col];
}

const double& Matrix::operator()(int row, int col) const {
    if (row < 0 || row >= rows_ || col < 0 || col >= cols_) {
        throw std::out_of_range("Matrix indices out of range");
    }
    return data_[row * cols_ + col];
}

Matrix Matrix::operator+(const Matrix& other) const {
    if (rows_ != other.rows_ || cols_ != other.cols_) {
        throw std::invalid_argument("Matrix dimensions must match for addition");
    }
    Matrix result(rows_, cols_);
    for (int i = 0; i < rows_ * cols_; i++) {
        result.data_[i] = data_[i] + other.data_[i];
    }
    return result;
}

Matrix Matrix::operator-(const Matrix& other) const {
    if (rows_ != other.rows_ || cols_ != other.cols_) {
        throw std::invalid_argument("Matrix dimensions must match for subtraction");
    }
    Matrix result(rows_, cols_);
    for (int i = 0; i < rows_ * cols_; i++) {
        result.data_[i] = data_[i] - other.data_[i];
    }
    return result;
}

Matrix Matrix::operator*(const Matrix& other) const {
    if (cols_ != other.rows_) {
        throw std::invalid_argument("Matrix dimensions must be compatible for multiplication");
    }
    Matrix result(rows_, other.cols_);
    for (int i = 0; i < rows_; i++) {
        for (int j = 0; j < other.cols_; j++) {
            double sum = 0.0;
            for (int k = 0; k < cols_; k++) {
                sum += (*this)(i, k) * other(k, j);
            }
            result(i, j) = sum;
        }
    }
    return result;
}

Matrix Matrix::transpose() const {
    Matrix result(cols_, rows_);
    for (int i = 0; i < rows_; i++) {
        for (int j = 0; j < cols_; j++) {
            result(j, i) = (*this)(i, j);
        }
    }
    return result;
}

int Matrix::getRows() const {
    return rows_;
}

int Matrix::getCols() const {
    return cols_;
}
