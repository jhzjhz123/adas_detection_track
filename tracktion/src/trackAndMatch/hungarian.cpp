#include"hungarian.h"

Hungarian::Hungarian(){}
Hungarian::~Hungarian(){
    delete[] this->row_mask;
    delete[] this->col_mask;
}

void Hungarian::minimize_along_direction(Eigen::Matrix<float,-1,-1,Eigen::RowMajor> &matrix, const bool over_columns) {
    const size_t outer_size = over_columns ? matrix.cols() : matrix.rows(),
                inner_size = over_columns ? matrix.rows() : matrix.cols();

    for ( size_t i = 0 ; i < outer_size ; i++ ) {
        double min = over_columns ? matrix(0, i) : matrix(i, 0);

        for ( size_t j = 1 ; j < inner_size && min > 0 ; j++ ) {
            min = XYZMIN(
            min,
            over_columns ? matrix(j, i) : matrix(i, j));
        }

        if ( min > 0 ) {
            for ( size_t j = 0 ; j < inner_size ; j++ ) {
            if ( over_columns ) {
                matrix(j, i) -= min;
            } else {
                matrix(i, j) -= min;
            }
            }
        }
    }
}


int Hungarian::step1() {
    const size_t rows = matrix.rows(),
                columns = matrix.cols();

    for ( size_t row = 0 ; row < rows ; row++ ) {
        for ( size_t col = 0 ; col < columns ; col++ ) {
        if ( 0 == matrix(row, col) ) {
            for ( size_t nrow = 0 ; nrow < row ; nrow++ )
            if ( STAR == mask_matrix(nrow,col) )
                goto next_column;

            mask_matrix(row,col) = STAR;
            goto next_row;
        }
        next_column:;
        }
        next_row:;
    }
    return 2;
}

int Hungarian::step2() {
    const size_t rows = matrix.rows(),
                columns = matrix.cols();
    size_t covercount = 0;

    for ( size_t row = 0 ; row < rows ; row++ )
        for ( size_t col = 0 ; col < columns ; col++ )
        if ( STAR == mask_matrix(row, col) ) {
            col_mask[col] = true;
            covercount++;
        }
    if ( covercount >= size ) {
        return 0;
    }
    return 3;
}

int Hungarian::step3() {
        if ( find_uncovered_in_matrix(0, saverow, savecol) ) {
            mask_matrix(saverow,savecol) = PRIME; // prime it.
        } else {
            return 5;
        }
        for ( unsigned int ncol = 0 ; ncol < matrix.cols() ; ncol++ ) {
            if ( mask_matrix(saverow,ncol) == STAR ) {
            row_mask[saverow] = true; //cover this row and
            col_mask[ncol] = false; // uncover the column containing the starred zero
            return 3; // repeat
            }
        }
        return 4; // no starred zero in the row containing this primed zero
    }

    int Hungarian::step4() {
    const size_t rows = matrix.rows(),
                columns = matrix.cols();
    std::list<std::pair<size_t,size_t> > seq;
    // use saverow, savecol from step 3.
    std::pair<size_t,size_t> z0(saverow, savecol);
    seq.insert(seq.end(), z0);
    // We have to find these two pairs:
    std::pair<size_t,size_t> z1(-1, -1);
    std::pair<size_t,size_t> z2n(-1, -1);

    size_t row, col = savecol;

    bool madepair;
    do {
        madepair = false;
        for ( row = 0 ; row < rows ; row++ ) {
        if ( mask_matrix(row,col) == STAR ) {
            z1.first = row;
            z1.second = col;
            if ( pair_in_list(z1, seq) ) {
            continue;
            }

            madepair = true;
            seq.insert(seq.end(), z1);
            break;
        }
        }

        if ( !madepair )
        break;

        madepair = false;

        for ( col = 0 ; col < columns ; col++ ) {
        if ( mask_matrix(row, col) == PRIME ) {
            z2n.first = row;
            z2n.second = col;
            if ( pair_in_list(z2n, seq) ) {
            continue;
            }
            madepair = true;
            seq.insert(seq.end(), z2n);
            break;
        }
        }
    } while ( madepair );

    for ( std::list<std::pair<size_t,size_t> >::iterator i = seq.begin() ;
        i != seq.end() ;
        i++ ) {
        // 2. Unstar each starred zero of the sequence.
        if ( mask_matrix(i->first,i->second) == STAR )
        mask_matrix(i->first,i->second) = NORMAL;

        // 3. Star each primed zero of the sequence,
        // thus increasing the number of starred zeros by one.
        if ( mask_matrix(i->first,i->second) == PRIME )
        mask_matrix(i->first,i->second) = STAR;
    }

    // 4. Erase all primes, uncover all columns and rows,
    for ( unsigned int row = 0 ; row < mask_matrix.rows() ; row++ ) {
        for ( unsigned int col = 0 ; col < mask_matrix.cols() ; col++ ) {
        if ( mask_matrix(row,col) == PRIME ) {
            mask_matrix(row,col) = NORMAL;
        }
        }
    }

    for ( size_t i = 0 ; i < rows ; i++ ) {
        row_mask[i] = false;
    }

    for ( size_t i = 0 ; i < columns ; i++ ) {
        col_mask[i] = false;
    }

    // and return to Step 2.
    return 2;
}

int Hungarian::step5() {
const size_t rows = matrix.rows(),
            columns = matrix.cols();
    double h = 100000;//xyzoylz std::numeric_limits<double>::max();
    for ( size_t row = 0 ; row < rows ; row++ ) {
        if ( !row_mask[row] ) {
            for ( size_t col = 0 ; col < columns ; col++ ) {
                if ( !col_mask[col] ) {
                    if ( h > matrix(row, col) && matrix(row, col) != 0 ) {
                        h = matrix(row, col);
                    }
                }
            }   
        }
    }

    for ( size_t row = 0 ; row < rows ; row++ ) {
        if ( row_mask[row] ) {
            for ( size_t col = 0 ; col < columns ; col++ ) {
                matrix(row, col) += h;
            }
        }
    }

    for ( size_t col = 0 ; col < columns ; col++ ) {
        if ( !col_mask[col] ) {
            for ( size_t row = 0 ; row < rows ; row++ ) {
                matrix(row, col) -= h;
            }
        }
    }

    return 3;
}


inline bool Hungarian::find_uncovered_in_matrix(const double item, size_t &row, size_t &col) const {
const size_t rows = matrix.rows(),
            columns = matrix.cols();

    for ( row = 0 ; row < rows ; row++ ) {
        if ( !row_mask[row] ) {
            for ( col = 0 ; col < columns ; col++ ) {
                if ( !col_mask[col] ) {
                if ( matrix(row,col) == item ) {
                    return true;
                }
                }
            }
        }
    }
    return false;
}

bool Hungarian::pair_in_list(const std::pair<size_t,size_t> &needle, const std::list<std::pair<size_t,size_t> > &haystack) {
    for ( std::list<std::pair<size_t,size_t> >::const_iterator i = haystack.begin() ; i != haystack.end() ; i++ ) {
        if ( needle == *i ) {
        return true;
        }
    }
    return false;
}


void Hungarian::initiate_resize_matrix(){
    this->cols = this->input_matrix.cols();
    this->rows = this->input_matrix.rows();
    this->size = XYZMAX(cols,rows);

    Eigen::Index maxRow, maxCol;
    double max = this->input_matrix.maxCoeff(&maxRow,&maxCol);
    this->matrix = MatrixXf(size,size) ;
    this->matrix.fill(max);
    this->matrix.topLeftCorner(rows,cols) = this->input_matrix.topLeftCorner(this->rows, this->cols);
    this->mask_matrix = MatrixXf::Zero(size,size);

}

void Hungarian::initiate_line_mask(){
    this->row_mask = new bool[size];
    this->col_mask = new bool[size];
    for ( size_t i = 0 ; i < this->size ; i++ ) {
        this->row_mask[i] = false;
    }

    for ( size_t i = 0 ; i < this->size ; i++ ) {
        this->col_mask[i] = false;
    }
}

Eigen::Matrix<float, -1, 2, Eigen::RowMajor> Hungarian::solve(Eigen::Matrix<float,-1,-1,Eigen::RowMajor> m){
    input_matrix = m;
    this->initiate_resize_matrix();
    this->initiate_line_mask();

    minimize_along_direction(input_matrix,  rows >= cols) ;
    minimize_along_direction(input_matrix, rows <  cols);

    int step = 1;
    while ( step ) {
        switch ( step ) {
        case 1:
            step = step1();
            // step is always 2
            break;
        case 2:
            step = step2();
            // step is always either 0 or 3
            break;
        case 3:
            step = step3();
            // step in [3, 4, 5]
            break;
        case 4:
            step = step4();
            // step is always 2
            break;
        case 5:
            step = step5();
            // step is always 3
            break;
        }
    }
    // Store results
    for ( size_t row = 0 ; row < size ; row++ ) {
        for ( size_t col = 0 ; col < size ; col++ ) {
            if ( mask_matrix(row, col) == STAR ) {
                matrix(row, col) = 0;
            } else {
                matrix(row, col) = -1;
            }
        }
    }
    output_matrix = matrix.topLeftCorner(rows, cols);

    std::vector<std::pair<int, int>> pairs;
    for (unsigned int row = 0; row < rows; row++) {
        for (unsigned int col = 0; col < cols; col++) {
            int tmp = (int)output_matrix(row, col);
            if (tmp == 0) pairs.push_back(std::make_pair(row, col));
        }
    }

    //
    int count = pairs.size();
    Eigen::Matrix<float, -1, 2, Eigen::RowMajor> re(count, 2);
    for (int i = 0; i < count; i++) {
        re(i, 0) = pairs[i].first;
        re(i, 1) = pairs[i].second;
    }

    delete [] row_mask;
    delete [] col_mask;
    
    return re;
};