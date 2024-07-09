class Matrix:
    def __init__(self, length: int):
        self.matrix = [[0]*length]*length
        self.row_count = length
        self.column_count = length

    def __init__(self, rows: int, cols: int) -> None:
        self.matrix = [[0]*cols]*rows
        self.row_count = rows
        self.column_count = cols

    def __init__(self, matrix) -> None:
        self.matrix = matrix

        self.row_count = len(self.matrix)
        self.column_count = len(self.matrix[0])

    def get_column(self, index: int):
        return self.get_columns()[index]

    def get_columns(self):
        columns = [[] for _ in range(self.column_count)]

        for row in self.matrix:
            for i, element in enumerate(row):
                columns[i].append(element)

        return columns

    def get_row(self, index: int):
        return self.matrix[index]

    def get_rows(self):
        return [x for x in self.matrix]

    def __repr__(self) -> str:
        matrix_str = ""
        for row in self.rows:
            matrix_str += " ".join(str(element) for element in row) + "\n"
        return matrix_str.strip()


a = [[2, 0, 6, 0], [3, 5, 0, 0], [0, 0, 3, 0], [1, 0, 0, 0]]
mt = Matrix(a)
print(mt.get_columns())
