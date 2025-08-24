%Name:        Alex Welke
%Date:        26/7/2025
%Description: Gets an Adjacency matrix of an extrusion

function Ablk = generateExtrusionAdj(A, num_profiles)
    %Size of matrx
    A_sz = size(A, 1);
                
    %Creates a list of n block adjacency matrices
    Ar = repmat(A, 1, num_profiles);
    
    %converts list of block matrices to cell array
    Acell = mat2cell(Ar, A_sz, ones(1, num_profiles)*A_sz);
    
    %Converts cell array into block diagonal matrix
    Ablk = blkdiag(Acell{:});
    
    %Adds block identity matrices along each of the off diagonals
    Ablk = Ablk + diag(ones(1,A_sz*(num_profiles-1)), A_sz)...
        + diag(ones(1,A_sz*(num_profiles-1)), -A_sz);
    
    %Creates a list of n block matrices containing adjacencies that relate to
    %edges that split square facets into triangle facets
    Ur = repmat(triu(A), 1, num_profiles-1);
    
    %Converts list of matrices into cell array
    Ucell = mat2cell(Ur, A_sz, ones(1, num_profiles-1)*A_sz);
    
    %Converts cell array into block diagonal matrix
    Ublk = blkdiag(Ucell{:});
    
    %Appends rows and columns of zeros onto the left and bottom sides of the
    %diagonal block matrix in order to move the diagonal to the 4-off diagonal
    Ublk = [zeros(A_sz*(num_profiles-1), A_sz), Ublk;...
        zeros(A_sz, A_sz*num_profiles)];
    
    %Adds the diagonal block matrix, as well as it's transpose, which populates
    %the lower left half of the matrix with the same elements
    Ablk = Ablk + Ublk + Ublk';

end