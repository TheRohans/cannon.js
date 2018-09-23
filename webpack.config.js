const DEBUG = (process.env.NODE_ENV !== 'production' && process.env.NODE_ENV !== 'test');

const webpack = require('webpack'),
  path = require('path'),
  WebpackStrip = require('strip-loader'),
  UglifyJSPlugin = require('uglifyjs-webpack-plugin');
  
const config = {
  entry: {
    dapao: './src/main.ts',
  },
  resolve: {
    // Add '.ts' and '.tsx' as a resolvable extension.
    extensions: [".ts", ".tsx"]
  },
  output: {
    path: path.join(__dirname, 'lib'),
    filename: '[name].js',
    chunkFilename: '[id].js',
    libraryTarget: 'umd',
  },
  plugins: [],
  module: {
    loaders: [
      {
        test: /\.ts?$/,
        loader: "ts-loader",
      },
      {
        test: /\.js$/,
        loader: 'babel-loader'
      },
    ]
  }
};

if (!DEBUG) {
  config.module.loaders.push(
    {
      test: /\.ts$/,
      loader: WebpackStrip.loader(
        'console.log',
        'Log.debug',
        'Log.time',
        'Log.timeEnd'
      )
    }
  );

  config.plugins.push(
    new UglifyJSPlugin({
      mangle: {
        except: ['$super', '$', 'exports', 'require']
      }
    })
  );
}

module.exports = config;
