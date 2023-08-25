import numpy as np
import scipy as sp


class TorqueModelV1:

    def optimize(self, df):
        initial_guess = np.zeros(7 * 2 + 1)
        initial_guess[-1] = 1
        coef = sp.optimize.minimize(self.err, initial_guess, args=(df))
        return [float(x) for x in coef.x]

    def err(self, betas, df):
        prediction = self.f(betas, df.iloc[:, df.columns != 'Torque'])
        expected = df['Torque'].to_numpy()

        err = (expected - prediction)
        RMSE = np.sqrt(np.mean(np.square(err)))

        return RMSE

    def calc_r2(self, betas, df):
        ''' Calculate the r squared value between the predicted and measured torque

        Args:
            betas (float[]): Coefficients of best fit
            ydata (Data Frame): Measured torque
            xdata (Data Frame): Data from muscles and joint angle
        Returns:
            r2 (float): r-squared measurement of prediction
            RMSE: root mean square error of prediction
        '''

        ydata = df['Torque']
        xdata = df.iloc[:, df.columns != 'Torque']

        err = (ydata - self.f(betas, xdata)).to_numpy()
        SSR = np.sum(np.square(err))
        norm = (ydata - np.mean(ydata)).to_numpy()
        SSN = np.sum(np.square(norm))

        RMSE = np.sqrt(SSR / xdata.iloc[:, 0].size)
        r2 = 1 - (SSR / SSN)

        return r2, RMSE

    @staticmethod
    def f(betas, X):
        JOINT_ANGLE = X.iloc[:, 0]
        TA = X.iloc[:, 1]
        GM = X.iloc[:, 2]
        SOL = X.iloc[:, 3]
        CONSTANT = np.ones(JOINT_ANGLE.size)

        f = (betas[0] * (TA - betas[1])).to_numpy() + (betas[2] * (TA * JOINT_ANGLE - betas[3])).to_numpy() \
            + (betas[4] * (GM - betas[5])).to_numpy() + (betas[6] * (GM * JOINT_ANGLE - betas[7])).to_numpy() \
            + (betas[8] * (SOL - betas[9])).to_numpy() + (betas[10] * (SOL * JOINT_ANGLE - betas[11])).to_numpy() \
            + (betas[12] * (JOINT_ANGLE - betas[13])).to_numpy() \
            + (betas[14] * CONSTANT)

        return f
