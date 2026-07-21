//
// -----------------------------------------------
// Provides Extreme Learning Machine algorithm.
// -----------------------------------------------
//
// Refer to:
// G.-B. Huang et al., “Extreme Learning Machine for Regression and Multiclass Classification,” IEEE TSMC, vol. 42, no.
// 2, pp. 513-529, 2012.
//
// Vladislavs D.
//

#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Cholesky>

using namespace std;
using namespace Eigen;

#ifndef ELM_H
#    define ELM_H

int compare(const void *a, const void *b);

// template <typename Derived>
MatrixXd buildTargetMatrix(double *Y, int nLabels);

// entry function to train the ELM model
// INPUT: X, Y, nhn, C
// OUTPUT: inW, bias, outW
template <typename Derived>
int elmTrain(double *             X,
             int                  dims,
             int                  nsmp,
             double *             Y,
             const int            nhn,
             const double         C,
             MatrixBase<Derived> &inW,
             MatrixBase<Derived> &bias,
             MatrixBase<Derived> &outW) {

    // map the samples into the matrix object
    MatrixXd mX = Map<MatrixXd>(X, dims, nsmp);

    // build target matrix
    MatrixXd mTargets = buildTargetMatrix(Y, nsmp);

    // generate random input weight matrix - inW
    inW = MatrixXd::Random(nhn, dims);

    // generate random bias vectors
    bias = MatrixXd::Random(nhn, 1);

    // compute the pre-H matrix
    MatrixXd preH = inW * mX + bias.replicate(1, nsmp);

    // compute hidden neuron output (sigmoid)
    MatrixXd H = (1 + (-preH.array()).exp()).cwiseInverse();

    // build matrices to solve Ax = b
    MatrixXd A = (MatrixXd::Identity(nhn, nhn)).array() * (1 / C) + (H * H.transpose()).array();
    MatrixXd b = H * mTargets.transpose();

    // solve the output weights as a solution to a system of linear equations
    outW = A.llt().solve(b);

    return 0;
}

// entry function to predict class labels using the trained ELM model on test data
// INPUT : X, inW, bias, outW
// OUTPUT : scores
template <typename Derived>
int elmPredict(double *             X,
               int                  dims,
               int                  nsmp,
               MatrixBase<Derived> &mScores,
               MatrixBase<Derived> &inW,
               MatrixBase<Derived> &bias,
               MatrixBase<Derived> &outW) {

    // map the sample into the Eigen's matrix object
    MatrixXd mX = Map<MatrixXd>(X, dims, nsmp);

    // build the pre-H matrix
    MatrixXd preH = inW * mX + bias.replicate(1, nsmp);

    // apply the activation function
    MatrixXd H = (1 + (-preH.array()).exp()).cwiseInverse();

    // compute output scores
    mScores = (H.transpose() * outW).transpose();

    return 0;
}

// --------------------------
// Helper functions
// --------------------------

// compares two integer values
// int compare( const void* a, const void *b ) {
//	return ( *(int *) a - *(int *) b );
//}

static int compare(const void *a, const void *b) {
    const double *da = (const double *)a;
    const double *db = (const double *)b;
    return (*da > *db) - (*da < *db);
}

// builds 1-of-K target matrix from labels array
// template <typename Derived>
static MatrixXd buildTargetMatrix(double *Y, int nLabels) {

    // make a temporary copy of the labels array
    double *tmpY = new double[nLabels];
    for (int i = 0; i < nLabels; i++) {
        tmpY[i] = Y[i];
    }

    // sort the array of labels
    qsort(tmpY, nLabels, sizeof(double), compare);

    // count unique labels
    int nunique = 1;
    for (int i = 0; i < nLabels - 1; i++) {
        if (tmpY[i] != tmpY[i + 1]) nunique++;
    }

    delete[] tmpY;

    MatrixXd targets(nunique, nLabels);
    targets.fill(0);

    // fill in the ones
    for (int i = 0; i < nLabels; i++) {
        int idx         = Y[i] - 1;
        targets(idx, i) = 1;
    }

    //// normalize the targets matrix values (-1/1)
    // targets *= 2;
    // targets.array() -= 1;

    return targets;
}

// ud_angle, lr_angle, pitch, roll, force, length, label
template <typename M>
M load_csv(const std::string &path) {
    std::ifstream indata;
    indata.open(path);
    std::string         line;
    std::vector<double> values;
    size_t              rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string       cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(
        values.data(), rows, values.size() / rows);
}

template <class Matrix>
void write_binary(const char *filename, const Matrix &matrix) {
    std::ofstream          out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
    typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();
    out.write((char *)(&rows), sizeof(typename Matrix::Index));
    out.write((char *)(&cols), sizeof(typename Matrix::Index));
    out.write((char *)matrix.data(), rows * cols * sizeof(typename Matrix::Scalar));
    out.close();
}

template <class Matrix>
void read_binary(const char *filename, Matrix &matrix) {
    std::ifstream          in(filename, std::ios::in | std::ios::binary);
    typename Matrix::Index rows = 0, cols = 0;
    in.read((char *)(&rows), sizeof(typename Matrix::Index));
    in.read((char *)(&cols), sizeof(typename Matrix::Index));
    matrix.resize(rows, cols);
    in.read((char *)matrix.data(), rows * cols * sizeof(typename Matrix::Scalar));
    in.close();
}

static const int hideNodes = 3000;

#endif
