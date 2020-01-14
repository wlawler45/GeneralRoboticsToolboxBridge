using System;
using System.Collections.Generic;
//using Microsoft.VisualStudio.TestTools.UnitTesting;
using TestGeneralRoboticsToolboxNET;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics;
using Bridge;
using Bridge.Html5;
using Bridge.QUnit;

namespace GeneralRoboticsToolboxTests
{
    [FileName("../samples/qunit/output/test.html")]
    public class UnitTest1
    {
        
        public void TestToolbox()
        {
            bool AlmostEquals(double val1, double val2)
            {
                if (Math.Abs(val1 - val2) < (1 * 10 ^ -8)) return true;
                return false;
            }
            bool AlmostEqualsMatrix(Matrix<double> val1, Matrix<double> val2)
            {
                for (int i = 0; i > val1.ColumnCount; i++)
                {
                    for (int j = 0; j > val1.RowCount; j++)
                    {
                        if (Math.Abs(val1[i, j] - val2[i, j]) > (1 * 10 ^ -8)) return false;
                    }
                }

                return true;
            }
            QUnit.Module("GeneralRoboticsToolbox");
            QUnit.Test("Method Hat()", (assert) =>
             {
                 assert.Expect(1);
                 Vector<double> k = Vector<double>.Build.DenseOfArray(new[] { 1.0, 2.0, 3.0 });
                 Matrix<double> khat = Matrix<double>.Build.Dense(3, 3);
                 khat[0, 1] = -3;
                 khat[0, 2] = 2;
                 khat[1, 0] = 3;
                 khat[1, 2] = -1;
                 khat[2, 0] = -2;
                 khat[2, 1] = 1;
                 Matrix<double> k_hat = GeneralRoboticsToolbox.Hat(k);
                 assert.Equal(k_hat, khat, "Hat didn't work");
             });


            QUnit.Test("Method Rot()", (assert) =>
            {
                assert.Expect(4);
                Vector<double> k = Vector<double>.Build.DenseOfArray(new[] { 1.0, 0, 0 });
                Matrix<double> rot1 = Matrix<double>.Build.Dense(3, 3);
                rot1[0, 0] = 1;
                rot1[0, 1] = 0;
                rot1[0, 2] = 0;
                rot1[1, 0] = 0;
                rot1[1, 1] = 0;
                rot1[1, 2] = 1;
                rot1[2, 0] = 0;
                rot1[2, 1] = 1;
                rot1[2, 2] = 0;



                rot1 = rot1.Transpose();


                Matrix<double> rot = GeneralRoboticsToolbox.Rot(k, Math.PI / 2);


                assert.Ok(AlmostEqualsMatrix(rot1, rot), "rot1 failed");

                Matrix<double> rot2 = Matrix<double>.Build.DenseOfRowArrays(new[] { 0, 0, -1.0 }, new[] { 0, 1.0, 0 }, new[] { 1.0, 0, 0 });
                Vector<double> k2 = Vector<double>.Build.DenseOfArray(new[] { 0, 1.0, 0 });
                rot2 = rot2.Transpose();
                Matrix<double> rot_2 = GeneralRoboticsToolbox.Rot(k2, Math.PI / 2);
                assert.Ok(AlmostEqualsMatrix(rot2, rot_2), "rot2 failed");



                Matrix<double> rot3 = Matrix<double>.Build.DenseOfRowArrays(new[] { 0, 1.0, 0 }, new[] { -1.0, 0, 0 }, new[] { 0, 0, 1.0 });
                Vector<double> k3 = Vector<double>.Build.DenseOfArray(new[] { 0, 0, 1.0 });
                rot3 = rot3.Transpose();
                Matrix<double> rot_3 = GeneralRoboticsToolbox.Rot(k3, Math.PI / 2);
                assert.Ok(AlmostEqualsMatrix(rot3, rot_3), "rot3 failed");

                Matrix<double> rot4 = Matrix<double>.Build.DenseOfRowArrays(new[] { -0.5057639, -0.1340537, 0.8521928 }, new[] { 0.6456962, -0.7139224, 0.2709081 }, new[] { 0.5720833, 0.6872731, 0.4476342 });
                Vector<double> k4 = Vector<double>.Build.DenseOfArray(new[] { 0.4490221, 0.30207945, 0.84090853 });
                rot4 = rot4.Transpose();
                Matrix<double> rot_4 = GeneralRoboticsToolbox.Rot(k4, 2.65949884);
                assert.Ok(AlmostEqualsMatrix(rot4, rot_4), "rot4 failed");


            });

        }
        
    }
}
