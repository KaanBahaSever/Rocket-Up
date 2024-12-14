#ifndef MATRIX_H
#define MATRIX_H

class Matrix {
private:
    int rows_;
    int cols_;
    double* data_;

public:
    Matrix(int rows, int cols);
    Matrix(const Matrix& other);
    ~Matrix();
    
    Matrix& operator=(const Matrix& other);
    double& operator()(int row, int col);
    const double& operator()(int row, int col) const;
    Matrix operator+(const Matrix& other) const;
    Matrix operator-(const Matrix& other) const;
    Matrix operator*(const Matrix& other) const;
    
    Matrix transpose() const;
    int getRows() const;
    int getCols() const;
};

#endif // MATRIX_H
