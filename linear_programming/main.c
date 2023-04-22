#include <stdio.h>

#define N_ROW 4 // 行数
#define N_COL 6 // 列数
#define N_VAR 2 // 変数の数

const double fmax = 1.0;
double min, p, d;
int i, j, k, x, y, flag;

void CalcLinearProgramming()
{
    // 係数行列
    static double a[N_ROW][N_COL] = {
        {1.0, 2.0, 1.0, 0.0, 0.0, 14.0},
        {1.0, 1.0, 0.0, 1.0, 0.0, 8.0},
        {3.0, 1.0, 0.0, 0.0, 1.0, 18.0},
        {-2.0, -3.0, 0.0, 0.0, 0.0, 0.0}};

    while (1)
    {
        // 列選択
        min = 9999;
        for (k = 0; k < N_COL - 1; k++)
        {
            if (a[N_ROW - 1][k] < min)
            {
                min = a[N_ROW - 1][k];
                y = k;
            }
        }
        if (min >= 0)
            break;

        // 行選択
        min = 9999;
        for (k = 0; k < N_ROW - 1; k++)
        {
            p = a[k][N_COL - 1] / a[k][y];
            if (a[k][y] > 0 && p < min)
            {
                min = p;
                x = k;
            }
        }

        // ピボット係数
        p = a[x][y];

        // ピボット係数を p で除算
        for (k = 0; k < N_COL; k++)
            a[x][k] = a[x][k] / p;

        // ピボット列の掃き出し
        for (k = 0; k < N_ROW; k++)
        {
            if (k != x)
            {
                d = a[k][y];
                for (j = 0; j < N_COL; j++)
                    a[k][j] = a[k][j] - d * a[x][j];
            }
        }
    }

    // 結果出力
    for (k = 0; k < N_VAR; k++)
    {
        flag = -1;
        for (j = 0; j < N_ROW; j++)
        {
            // ==== 2016-11-14 UPDATE ===>
            // if (a[j][k] == 1) flag = j;
            if (a[j][k] == 1)
            {
                flag = j;
            }
            else if (flag != -1 && a[j][k] != 0)
            {
                flag = -1;
                break;
            }
            // <=== 2016-11-14 UPDATE ====
        }
        if (flag != -1)
            printf("x%d = %8.4f\n", k, a[flag][N_COL - 1]);
        else
            printf("x%d = %8.4f\n", k, 0.0);
    }
    printf("z  = %8.4f\n", a[N_ROW - 1][N_COL - 1]);
}

int main()
{
    CalcLinearProgramming();

    return 0;
}