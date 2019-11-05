const path = require('path');
const webpack = require('webpack');
const MiniCssExtractPlugin = require("mini-css-extract-plugin");

module.exports = {
  entry: {
    app: path.join(__dirname, 'index.js')
  },
  devtool: 'source-map',
  module: {
    rules: [
      {
        test: /\.css$/,
        loader: [
          {
            loader: MiniCssExtractPlugin.loader,
            options: {
              hmr: process.env.NODE_ENV === 'development',
            },
          },
          'css-loader',
        ]
      },
    ]
  },
  devServer: {
    before: function(app, server, compiler) {
      app.use('USA-road-t.NY.co', function(req, res, next) {
        res.header('Content-Type' , 'text/plain');
        next();
      });
    },
    contentBase: [__dirname, path.resolve(__dirname, '..')],
    compress: false,
    port: 3000,
  },
  plugins: [
    new webpack.HotModuleReplacementPlugin(),
    new webpack.NoEmitOnErrorsPlugin(),
    new MiniCssExtractPlugin({
      filename: "index.css",
      ignoreOrder: true
    })
  ]
};