# Phase 1: Project Setup & CDK Infrastructure

## Goal

Create the CDK stack that deploys all AWS infrastructure: VPC, EC2 G6e.2xlarge with Isaac Sim AMI, IAM role with IoT Core permissions, and security group.

## Prerequisites

- AWS account with G6e instance quota in target region
- CDK CLI installed (`npm install -g aws-cdk`)
- Node.js 18+

## Tasks

### 1.1 Initialize CDK Project

```
/infra
├── bin/app.ts              # CDK app entry point
├── lib/isaac-sim-stack.ts  # Single stack
├── package.json
├── tsconfig.json
└── cdk.json
```

- `npx cdk init app --language typescript` in `/infra`
- Add dependencies: `aws-cdk-lib`, `constructs`

### 1.2 Create config.json

Project-wide config at repo root:

```json
{
  "project": {
    "name": "isaac-sim-dt",
    "environment": "dev"
  },
  "aws": {
    "region": "us-east-1"
  },
  "iot": {
    "deviceId": "arm-001",
    "topicPrefix": "dt/lerobot",
    "pollingRateHz": 10
  },
  "ec2": {
    "instanceType": "g6e.2xlarge",
    "amiId": "prodview-bl35herdyozhw",
    "volumeSizeGb": 100,
    "clientIpCidr": "0.0.0.0/0"
  },
  "dcv": {
    "port": 8443
  }
}
```

Note: `clientIpCidr` should be set to the user's actual IP before deploy.

### 1.3 CDK Stack — VPC

- VPC `10.0.0.0/16`, single AZ, public subnet only
- Internet Gateway (created automatically with public subnet)
- No NAT Gateway needed

```typescript
const vpc = new ec2.Vpc(this, 'Vpc', {
  maxAzs: 1,
  natGateways: 0,
  subnetConfiguration: [{
    name: 'Public',
    subnetType: ec2.SubnetType.PUBLIC,
  }],
});
```

### 1.4 CDK Stack — Security Group

```typescript
const sg = new ec2.SecurityGroup(this, 'IsaacSimSG', {
  vpc,
  description: 'Isaac Sim DT - DCV + SSH access',
  allowAllOutbound: true,
});
sg.addIngressRule(
  ec2.Peer.ipv4(config.ec2.clientIpCidr),
  ec2.Port.tcp(config.dcv.port),
  'NICE DCV access'
);
sg.addIngressRule(
  ec2.Peer.ipv4(config.ec2.clientIpCidr),
  ec2.Port.tcp(22),
  'SSH access'
);
```

### 1.5 CDK Stack — IAM Role

EC2 instance role with:

```typescript
const role = new iam.Role(this, 'EC2Role', {
  assumedBy: new iam.ServicePrincipal('ec2.amazonaws.com'),
  managedPolicies: [
    iam.ManagedPolicy.fromAwsManagedPolicyName('AmazonSSMManagedInstanceCore'),
  ],
});

// IoT Core permissions for MQTT/WebSocket with SigV4
role.addToPolicy(new iam.PolicyStatement({
  effect: iam.Effect.ALLOW,
  actions: ['iot:Connect'],
  resources: [`arn:aws:iot:${region}:${account}:client/isaac-sim-dt-*`],
}));
role.addToPolicy(new iam.PolicyStatement({
  effect: iam.Effect.ALLOW,
  actions: ['iot:Subscribe'],
  resources: [`arn:aws:iot:${region}:${account}:topicfilter/dt/lerobot/${config.iot.deviceId}/*`],
}));
role.addToPolicy(new iam.PolicyStatement({
  effect: iam.Effect.ALLOW,
  actions: ['iot:Receive'],
  resources: [`arn:aws:iot:${region}:${account}:topic/dt/lerobot/${config.iot.deviceId}/*`],
}));
role.addToPolicy(new iam.PolicyStatement({
  effect: iam.Effect.ALLOW,
  actions: ['iot:DescribeEndpoint'],
  resources: ['*'],
}));
```

### 1.6 CDK Stack — EC2 Instance

- Use marketplace AMI `prodview-bl35herdyozhw`
- Attach UserData script (Phase 2b)
- Key pair for SSH (or SSM-only)
- 100 GB gp3 root volume

```typescript
const instance = new ec2.Instance(this, 'IsaacSim', {
  vpc,
  instanceType: new ec2.InstanceType(config.ec2.instanceType),
  machineImage: ec2.MachineImage.lookup({
    name: '*Isaac*Sim*',  // or use specific AMI ID
    owners: ['aws-marketplace'],
  }),
  securityGroup: sg,
  role,
  vpcSubnets: { subnetType: ec2.SubnetType.PUBLIC },
  blockDevices: [{
    deviceName: '/dev/sda1',
    volume: ec2.BlockDeviceVolume.ebs(config.ec2.volumeSizeGb, {
      volumeType: ec2.EbsDeviceVolumeType.GP3,
    }),
  }],
  associatePublicIpAddress: true,
});
```

### 1.7 CDK Outputs

```typescript
new CfnOutput(this, 'InstanceId', { value: instance.instanceId });
new CfnOutput(this, 'PublicIp', { value: instance.instancePublicIp });
new CfnOutput(this, 'DCVUrl', {
  value: `https://${instance.instancePublicDnsName}:${config.dcv.port}`,
});
```

## Acceptance Criteria

- [ ] `npx cdk synth` produces valid CloudFormation
- [ ] Stack creates VPC, SG, IAM role, EC2 instance
- [ ] IAM role has IoT Core permissions scoped to `dt/lerobot/arm-001/*`
- [ ] Security group restricts inbound to configured client IP
- [ ] EC2 uses Isaac Sim marketplace AMI with 100GB gp3 volume
