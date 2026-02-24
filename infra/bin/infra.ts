#!/usr/bin/env node
import * as cdk from 'aws-cdk-lib';
import { IsaacSimDTStack } from '../lib/isaac-sim-stack';
import * as fs from 'fs';
import * as path from 'path';

const config = JSON.parse(
  fs.readFileSync(path.join(__dirname, '../../config.json'), 'utf-8')
);

const app = new cdk.App();
new IsaacSimDTStack(app, 'IsaacSimDTStack', {
  env: {
    account: process.env.CDK_DEFAULT_ACCOUNT,
    region: config.aws.region,
  },
  description: 'Real-time digital twin of SO-101 robotic arm using Isaac Sim',
});
